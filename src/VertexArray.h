#pragma once

#include <mutex>
#include <vector>
#include "d3dx12.h"
#include <d3d12.h>
#include <windows.h>
#include <wrl.h>

#include "QuadTree.h"
#include "util.h"

#include "d3d12.h"

using Microsoft::WRL::ComPtr;

const int kMaxUploadBufferCapacity = 50000000;
const int kMaxMeshSize = 100000;
const int kNewUploadBufferThreshold = kMaxUploadBufferCapacity - kMaxMeshSize;

struct UploadBuffer {
  ComPtr<ID3D12Resource> buffer;
  D3D12_VERTEX_BUFFER_VIEW bufferView;
  std::vector<float> vertices;
  int numUploaded;
};

class VertexArray {
 public:
  VertexArray(ComPtr<ID3D12Device>& device) : device_(device) {
    currentBufferId_ = -1;
  }

  ~VertexArray() { }

  void Init() { NewUploadBuffer(); }

  VertexRef AddVertices(QuadTree* cell, const std::vector<float>& vertices) {
    std::unique_lock lock(mutex_);

    // 136 931 328
    if (buffers_.back().numUploaded + vertices_.size() >
            kNewUploadBufferThreshold ||
        vertices_.size() > kMaxUploadBufferCapacity / 5) {
      SubmitToGpu();
    }
    int bufferId = buffers_.size() - 1;
    int arraySize = vertices_.size();
    int vertexIndex = buffers_.back().numUploaded + arraySize;
    vertices_.insert(vertices_.end(), vertices.begin(), vertices.end());

    VertexRef vertexRef;
    vertexRef.bufferId = bufferId;
    vertexRef.vertexIndex = vertexIndex;
    vertexRef.meshSize = vertices.size();
    cells_.emplace_back(cell, vertexRef);

    return vertexRef;
  }

  void FlushToGpu() {
    std::unique_lock lock(mutex_);

    SubmitToGpu();
  }

  void MaybeFlushToGpu() {
    std::unique_lock lock(mutex_);

    if (vertices_.size() > 300000) {
      SubmitToGpu();
    }
  }

  void StartRendering(ComPtr<ID3D12GraphicsCommandList>& commandList) {
    commandList_ = commandList;
    currentBufferId_ = -1;
  }

  void Render(const VertexRef& vertex_ref) {
    if (vertex_ref.bufferId != currentBufferId_) {
      currentBufferId_ = vertex_ref.bufferId;
      commandList_->IASetVertexBuffers(0, 1,
                                       &buffers_[currentBufferId_].bufferView);
    }
    commandList_->DrawInstanced(vertex_ref.meshSize / kFloatsPerPoint, 1,
                                vertex_ref.vertexIndex / kFloatsPerPoint, 0);
  }

  void StopRendering() { currentBufferId_ = -1; }

 protected:
  void CopyToGpu() {
    dprintf("Copying buffer %d to gpu.\n", buffers_.size());
    ComPtr<ID3D12Resource>& vertexBuffer = buffers_.back().buffer;

    UINT8* mappedBuffer;
    CD3DX12_RANGE readRange(0, 0);
    vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&mappedBuffer));
    memcpy((float*)mappedBuffer, &vertices_[0],
           sizeof(float) * vertices_.size());
    vertexBuffer->Unmap(0, nullptr);

    vertices_.clear();
  }

  void SubmitToGpu() {
    int floatsToUpload = vertices_.size();
    if (floatsToUpload == 0) {
      return;
    }

    UploadBuffer& uploadBuffer = buffers_.back();

    float* mappedBuffer;
    CD3DX12_RANGE readRange(0, 0);
    uploadBuffer.buffer->Map(0, &readRange,
                             reinterpret_cast<void**>(&mappedBuffer));
    memcpy(&mappedBuffer[uploadBuffer.numUploaded], &vertices_[0],
           sizeof(float) * vertices_.size());
    uploadBuffer.buffer->Unmap(0, nullptr);

    uploadBuffer.numUploaded += vertices_.size();
    vertices_.clear();

    for (const std::pair<QuadTree*, VertexRef>& cell : cells_) {
      cell.first->NotifyCommitted(cell.second);
    }
    cells_.clear();

    if (buffers_.back().numUploaded + floatsToUpload >
        kNewUploadBufferThreshold) {
      NewUploadBuffer();
    }
  }

  void NewUploadBuffer() {
    dprintf("New upload buffer %d.\n", buffers_.size());

    buffers_.emplace_back();
    UploadBuffer& uploadBuffer = buffers_.back();
    uploadBuffer.numUploaded = 0;

    device_->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(sizeof(float) *
                                       kMaxUploadBufferCapacity),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&uploadBuffer.buffer));

    D3D12_VERTEX_BUFFER_VIEW& vertexBufferView = uploadBuffer.bufferView;
    vertexBufferView.BufferLocation =
        uploadBuffer.buffer->GetGPUVirtualAddress();
    vertexBufferView.StrideInBytes = kFloatsPerPoint * sizeof(float);
    vertexBufferView.SizeInBytes = sizeof(float) * kMaxUploadBufferCapacity;
  }

 private:
  // Adding vertices is thread safe
  std::mutex mutex_;
  std::vector<float> vertices_;
  std::vector<std::pair<QuadTree*, VertexRef>> cells_;
  std::vector<UploadBuffer> buffers_;

  // Device data
  ComPtr<ID3D12Device>& device_;

  // Rendering Data
  ComPtr<ID3D12GraphicsCommandList> commandList_;
  int currentBufferId_;
};
