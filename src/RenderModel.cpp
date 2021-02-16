#include "RenderModel.h"

#include <vector>

#include "MainApplication.h"

// Create/destroy D3D12 Render Models
RenderModel::RenderModel(const std::string& renderModelName)
    : modelName_(renderModelName) {
  memset(constantBufferData_, 0, sizeof(constantBufferData_));
}

RenderModel::~RenderModel() { Cleanup(); }

// Allocates and populates the D3D12 resources for a render model
bool RenderModel::BInit(ID3D12Device* device,
                        ID3D12GraphicsCommandList* commandList,
                        ID3D12DescriptorHeap* cbvSrvHeap,
                        vr::TrackedDeviceIndex_t trackedDeviceIndex,
                        const vr::RenderModel_t& vrModel,
                        const vr::RenderModel_TextureMap_t& vrDiffuseTexture) {
  trackedDeviceIndex_ = trackedDeviceIndex;
  cbvSrvHeap_ = cbvSrvHeap;

  // Create and populate the vertex buffer
  {
    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(sizeof(vr::RenderModel_Vertex_t) *
                                       vrModel.unVertexCount),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&vertexBuffer_));

    UINT8* mappedBuffer;
    CD3DX12_RANGE readRange(0, 0);
    vertexBuffer_->Map(0, &readRange, reinterpret_cast<void**>(&mappedBuffer));
    memcpy(mappedBuffer, vrModel.rVertexData,
           sizeof(vr::RenderModel_Vertex_t) * vrModel.unVertexCount);
    vertexBuffer_->Unmap(0, nullptr);

    vertexBufferView_.BufferLocation = vertexBuffer_->GetGPUVirtualAddress();
    vertexBufferView_.StrideInBytes = sizeof(vr::RenderModel_Vertex_t);
    vertexBufferView_.SizeInBytes =
        sizeof(vr::RenderModel_Vertex_t) * vrModel.unVertexCount;
  }

  // Create and populate the index buffer
  {
    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(sizeof(uint16_t) *
                                       vrModel.unTriangleCount * 3),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&indexBuffer_));

    UINT8* mappedBuffer;
    CD3DX12_RANGE readRange(0, 0);
    indexBuffer_->Map(0, &readRange, reinterpret_cast<void**>(&mappedBuffer));
    memcpy(mappedBuffer, vrModel.rIndexData,
           sizeof(uint16_t) * vrModel.unTriangleCount * 3);
    indexBuffer_->Unmap(0, nullptr);

    indexBufferView_.BufferLocation = indexBuffer_->GetGPUVirtualAddress();
    indexBufferView_.Format = DXGI_FORMAT_R16_UINT;
    indexBufferView_.SizeInBytes =
        sizeof(uint16_t) * vrModel.unTriangleCount * 3;
  }

  // create and populate the texture
  {
    int imageWidth = vrDiffuseTexture.unWidth;
    int imageHeight = vrDiffuseTexture.unHeight;
    std::vector<D3D12_SUBRESOURCE_DATA> mipLevelData;

    UINT8* baseData = new UINT8[imageWidth * imageHeight * 4];
    memcpy(baseData, vrDiffuseTexture.rubTextureMapData,
           sizeof(UINT8) * imageWidth * imageHeight * 4);
    D3D12_SUBRESOURCE_DATA textureData = {};
    textureData.pData = &baseData[0];
    textureData.RowPitch = imageWidth * 4;
    textureData.SlicePitch = textureData.RowPitch * imageHeight;
    mipLevelData.push_back(textureData);

    // Generate mipmaps for the image
    int prevImageIndex = 0;
    int mipWidth = imageWidth;
    int mipHeight = imageHeight;

    while (mipWidth > 1 && mipHeight > 1) {
      UINT8* newImage;
      MainApplication::GenMipMapRGBA((UINT8*)mipLevelData[prevImageIndex].pData,
                                     &newImage, mipWidth, mipHeight, &mipWidth,
                                     &mipHeight);

      D3D12_SUBRESOURCE_DATA mipData = {};
      mipData.pData = newImage;
      mipData.RowPitch = mipWidth * 4;
      mipData.SlicePitch = textureData.RowPitch * mipHeight;
      mipLevelData.push_back(mipData);

      prevImageIndex++;
    }

    D3D12_RESOURCE_DESC textureDesc = {};
    textureDesc.MipLevels = (UINT16)mipLevelData.size();
    textureDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    textureDesc.Width = imageWidth;
    textureDesc.Height = imageHeight;
    textureDesc.Flags = D3D12_RESOURCE_FLAG_NONE;
    textureDesc.DepthOrArraySize = 1;
    textureDesc.SampleDesc.Count = 1;
    textureDesc.SampleDesc.Quality = 0;
    textureDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;

    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT), D3D12_HEAP_FLAG_NONE,
        &textureDesc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
        IID_PPV_ARGS(&texture_));

    // Create shader resource view
    CD3DX12_CPU_DESCRIPTOR_HANDLE srvHandle(
        cbvSrvHeap->GetCPUDescriptorHandleForHeapStart());
    srvHandle.Offset(SRV_TEXTURE_RENDER_MODEL0 + trackedDeviceIndex,
                     device->GetDescriptorHandleIncrementSize(
                         D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV));
    device->CreateShaderResourceView(texture_.Get(), nullptr, srvHandle);

    const UINT64 uploadBufferSize =
        GetRequiredIntermediateSize(texture_.Get(), 0, textureDesc.MipLevels);

    // Create the GPU upload buffer.
    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(uploadBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&textureUploadHeap_));

    UpdateSubresources(commandList, texture_.Get(), textureUploadHeap_.Get(), 0,
                       0, mipLevelData.size(), &mipLevelData[0]);
    commandList->ResourceBarrier(
        1, &CD3DX12_RESOURCE_BARRIER::Transition(
               texture_.Get(), D3D12_RESOURCE_STATE_COPY_DEST,
               D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE));

    // Free mip pointers
    for (size_t mip = 0; mip < mipLevelData.size(); mip++) {
      delete[] mipLevelData[mip].pData;
    }
  }

  // Create a constant buffer to hold the transform (one for each eye)
  {
    device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(1024 * 64),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&constantBuffer_));

    // Keep as persistently mapped buffer, store left eye in first 256 bytes,
    // right eye in second
    UINT8* buffer;
    CD3DX12_RANGE readRange(0, 0);
    constantBuffer_->Map(0, &readRange, reinterpret_cast<void**>(&buffer));
    // Left eye to first 256 bytes, right eye to second 256 bytes
    constantBufferData_[0] = buffer;
    constantBufferData_[1] = buffer + 256;

    // Left eye CBV
    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvLeftEyeHandle(
        cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
    cbvLeftEyeHandle.Offset(CBV_LEFT_EYE_RENDER_MODEL0 + trackedDeviceIndex_,
                            device->GetDescriptorHandleIncrementSize(
                                D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV));
    D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
    cbvDesc.BufferLocation = constantBuffer_->GetGPUVirtualAddress();
    cbvDesc.SizeInBytes = (sizeof(Matrix4) + 255) & ~255;  // Pad to 256 bytes
    device->CreateConstantBufferView(&cbvDesc, cbvLeftEyeHandle);

    // Right eye CBV
    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvRightEyeHandle(
        cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
    cbvRightEyeHandle.Offset(CBV_RIGHT_EYE_RENDER_MODEL0 + trackedDeviceIndex_,
                             device->GetDescriptorHandleIncrementSize(
                                 D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV));
    cbvDesc.BufferLocation += 256;
    device->CreateConstantBufferView(&cbvDesc, cbvRightEyeHandle);
  }

  vertexCount_ = vrModel.unTriangleCount * 3;

  return true;
}

// Frees the D3D12 resources for a render model
void RenderModel::Cleanup() {
  // TBD
}

//  Draws the render model
void RenderModel::Draw(vr::EVREye eye, ID3D12GraphicsCommandList* commandList,
                       UINT cbvSrvDescriptorSize, const Matrix4& matMVP) {
  // Update the CB with the transform
  memcpy(constantBufferData_[eye], &matMVP, sizeof(matMVP));

  // Bind the CB
  int startOffset = (eye == vr::Eye_Left) ? CBV_LEFT_EYE_RENDER_MODEL0
                                          : CBV_RIGHT_EYE_RENDER_MODEL0;
  CD3DX12_GPU_DESCRIPTOR_HANDLE cbvHandle(
      cbvSrvHeap_->GetGPUDescriptorHandleForHeapStart());
  cbvHandle.Offset(startOffset + trackedDeviceIndex_, cbvSrvDescriptorSize);
  commandList->SetGraphicsRootDescriptorTable(0, cbvHandle);

  // Bind the texture
  CD3DX12_GPU_DESCRIPTOR_HANDLE srvHandle(
      cbvSrvHeap_->GetGPUDescriptorHandleForHeapStart());
  srvHandle.Offset(SRV_TEXTURE_RENDER_MODEL0 + trackedDeviceIndex_,
                   cbvSrvDescriptorSize);
  commandList->SetGraphicsRootDescriptorTable(1, srvHandle);

  // Bind the VB/IB and draw
  commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  commandList->IASetVertexBuffers(0, 1, &vertexBufferView_);
  commandList->IASetIndexBuffer(&indexBufferView_);
  commandList->DrawIndexedInstanced(vertexCount_, 1, 0, 0, 0);
}
