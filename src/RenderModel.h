#pragma once

#include <string>
#include <openvr.h>

#include "d3dx12.h"
#include <d3d12.h>
#include <windows.h>
#include <wrl.h>
#include <Matrices.h>

using Microsoft::WRL::ComPtr;

class RenderModel {
 public:
  RenderModel(const std::string& renderModelName);
  ~RenderModel();

  bool BInit(ID3D12Device* device, ID3D12GraphicsCommandList* commandList,
             ID3D12DescriptorHeap* cbvSrvHeap,
             vr::TrackedDeviceIndex_t trackedDeviceIndex,
             const vr::RenderModel_t& vrModel,
             const vr::RenderModel_TextureMap_t& vrDiffuseTexture);
  void Cleanup();
  void Draw(vr::EVREye eye, ID3D12GraphicsCommandList* commandList,
            UINT cbvSrvDescriptorSize, const Matrix4& mvp);
  const std::string& GetName() const { return modelName_; }

 private:
  ComPtr<ID3D12Resource> vertexBuffer_;
  D3D12_VERTEX_BUFFER_VIEW vertexBufferView_;
  ComPtr<ID3D12Resource> indexBuffer_;
  D3D12_INDEX_BUFFER_VIEW indexBufferView_;
  ComPtr<ID3D12Resource> texture_;
  ComPtr<ID3D12Resource> textureUploadHeap_;
  ComPtr<ID3D12Resource> constantBuffer_;
  UINT8* constantBufferData_[2];
  size_t vertexCount_;
  vr::TrackedDeviceIndex_t trackedDeviceIndex_;
  ID3D12DescriptorHeap* cbvSrvHeap_;
  std::string modelName_;
};