#pragma once

#include <D3Dcompiler.h>
#include <vector>
#include <d3d12.h>
#include <d3dx12.h>
#include <dxgi1_4.h>
#include <SDL.h>
#include <openvr.h>
#include <windows.h>
#include <wrl.h>
#include <Matrices.h>

#include "QuadTree.h"
#include "ExtendList.h"
#include "RenderModel.h"
#include "util.h"

#include "d3d12.h"

using Microsoft::WRL::ComPtr;

// Slots in the RenderTargetView descriptor heap
enum RTVIndex {
  RTV_LEFT_EYE = 0,
  RTV_RIGHT_EYE,
  RTV_SWAPCHAIN0,
  RTV_SWAPCHAIN1,
  NUM_RTVS
};

// Slots in the ConstantBufferView/ShaderResourceView descriptor heap
enum CBVSRVIndex {
  CBV_LEFT_EYE = 0,
  CBV_RIGHT_EYE,
  CBV_COLOR_LOOKUP_LEFT,
  CBV_COLOR_LOOKUP_RIGHT,
  SRV_LEFT_EYE,
  SRV_RIGHT_EYE,
  SRV_TEXTURE_MAP,
  // Slot for texture in each possible render model
  SRV_TEXTURE_RENDER_MODEL0,
  SRV_TEXTURE_RENDER_MODEL_MAX =
      SRV_TEXTURE_RENDER_MODEL0 + vr::k_unMaxTrackedDeviceCount,
  // Slot for transform in each possible rendermodel
  CBV_LEFT_EYE_RENDER_MODEL0,
  CBV_LEFT_EYE_RENDER_MODEL_MAX =
      CBV_LEFT_EYE_RENDER_MODEL0 + vr::k_unMaxTrackedDeviceCount,
  CBV_RIGHT_EYE_RENDER_MODEL0,
  CBV_RIGHT_EYE_RENDER_MODEL_MAX =
      CBV_RIGHT_EYE_RENDER_MODEL0 + vr::k_unMaxTrackedDeviceCount,
  NUM_SRV_CBVS
};

static const int kFrameCount = 2;  // Swapchain depth

class MainApplication {
 public:
  MainApplication(int argc, char* argv[]);
  virtual ~MainApplication();

  bool BInit();
  bool BInitD3D12();
  bool BInitCompositor();

  void SetupRenderModels();

  void Shutdown();
  bool IsShuttingDown();

  void RunMainLoop();
  bool HandleInput();
  void ProcessVREvent(const vr::VREvent_t& event);
  void RenderFrame();

  static void GenMipMapRGBA(const UINT8* src, UINT8** dst, int srcWidth,
                            int srcHeight, int* dstWidthOut, int* dstHeightOut);

  void SetupMandelScene();
  void CreateFrustumPlanes(vr::Hmd_Eye eye, std::vector<Vector4>* planes,
                           Vector3* cameraPos);

  void UpdateControllerAxes();

  bool SetupStereoRenderTargets();
  void SetupCompanionWindow();
  void SetupCameras();

  void RenderStereoTargets();
  void RenderCompanionWindow();
  void RenderScene(vr::Hmd_Eye eye);

  Matrix4 GetHMDMatrixProjectionEye(vr::Hmd_Eye eye);
  Matrix4 GetHMDMatrixPoseEye(vr::Hmd_Eye eye);
  Matrix4 GetCurrentViewProjectionMatrix(vr::Hmd_Eye eye);
  void UpdateHMDMatrixPose();
  void UpdateLodLevels();

  Matrix4 ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t& matPose);

  bool CreateAllShaders();

  void SetupRenderModelForTrackedDevice(
      vr::TrackedDeviceIndex_t trackedDeviceIndex);
  RenderModel* FindOrLoadRenderModel(
      vr::TrackedDeviceIndex_t trackedDeviceIndex, const char* renderModelName);

 private:
  bool debugD3D12_;
  bool verbose_;
  bool perf_;
  bool vBlank_;
  int msaaSampleCount_;
  // Optional scaling factor to render with supersampling (defaults off, use
  // -scale)
  float superSampleScale_;

  bool isShuttingDown_;
  std::mutex shutdownMutex_;

  vr::IVRSystem* hmd_;
  vr::IVRRenderModels* ivrRenderModels_;
  std::string driver_;
  std::string display_;
  vr::TrackedDevicePose_t trackedDevicePose_[vr::k_unMaxTrackedDeviceCount];
  Matrix4 devicePose_[vr::k_unMaxTrackedDeviceCount];
  bool showTrackedDevice_[vr::k_unMaxTrackedDeviceCount];
  vr::VRControllerState_t controllerState_[vr::k_unMaxTrackedDeviceCount];

  // SDL bookkeeping
  SDL_Window* companionWindow_;
  uint32_t companionWindowWidth_;
  uint32_t companionWindowHeight_;

  int trackedControllerCount_;
  int trackedControllerCountLast_;
  int validPoseCount_;
  int validPoseCountLast_;
  bool showMandelPlane_;

  std::string poseClasses_;  // what classes we saw poses for this frame
  char devClassChar_[vr::k_unMaxTrackedDeviceCount];  // for each device, a
                                                      // character representing
                                                      // its class

  float nearClip_;
  float farClip_;

  unsigned int vertCount_;
  unsigned int companionWindowIndexSize_;

  // Mandel scene
  std::unique_ptr<thread_pool> threadPool_;
  unsigned int mandelVertCount_;
  QuadTree* quadTree_;
  int mandelResolution_;
  Matrix4 mat4DebugPos_;
  Vector3 debugFrustumPos_;
  std::vector<Vector4> debugFrustumPlanes_;
  int maxLevel_;
  std::vector<float> lodLevels_;
  ExtendList extendList_;

  // D3D12 members
  UINT frameIndex_;
  HANDLE fenceEvent_;
  ComPtr<ID3D12Fence> fence_;
  UINT64 fenceValues_[kFrameCount];
  ComPtr<ID3D12Device> device_;
  ComPtr<IDXGISwapChain3> swapChain_;
  ComPtr<ID3D12Resource> swapChainRenderTarget_[kFrameCount];
  ComPtr<ID3D12CommandQueue> commandQueue_;
  ComPtr<ID3D12CommandAllocator> commandAllocators_[kFrameCount];
  ComPtr<ID3D12GraphicsCommandList> commandList_;
  ComPtr<ID3D12DescriptorHeap> cbvSrvHeap_;
  ComPtr<ID3D12DescriptorHeap> rtvHeap_;
  ComPtr<ID3D12DescriptorHeap> dsvHeap_;
  ComPtr<ID3D12RootSignature> rootSignature_;
  ComPtr<ID3D12PipelineState> mandelPipelineState_;
  ComPtr<ID3D12PipelineState> companionPipelineState_;
  ComPtr<ID3D12PipelineState> axesPipelineState_;
  ComPtr<ID3D12PipelineState> scenePipelineState_;
  ComPtr<ID3D12PipelineState> renderModelPipelineState_;
  ComPtr<ID3D12Resource> sceneConstantBuffer_;
  D3D12_CPU_DESCRIPTOR_HANDLE sceneConstantBufferView_[2];
  UINT8* sceneConstantBufferData_[2];
  ComPtr<ID3D12Resource> colorLookupConstantBuffer_;
  UINT rtvDescriptorSize_;
  UINT dsvDescriptorSize_;
  UINT cbvSrvDescriptorSize_;

  ComPtr<ID3D12Resource> sceneVertexBuffer_;
  D3D12_VERTEX_BUFFER_VIEW sceneVertexBufferView_;
  ComPtr<ID3D12Resource> texture_;
  ComPtr<ID3D12Resource> textureUploadHeap_;
  D3D12_CPU_DESCRIPTOR_HANDLE textureShaderResourceView_;
  ComPtr<ID3D12Resource> companionWindowVertexBuffer_;
  D3D12_VERTEX_BUFFER_VIEW companionWindowVertexBufferView_;
  ComPtr<ID3D12Resource> companionWindowIndexBuffer_;
  D3D12_INDEX_BUFFER_VIEW companionWindowIndexBufferView_;
  ComPtr<ID3D12Resource> controllerAxisVertexBuffer_;
  D3D12_VERTEX_BUFFER_VIEW controllerAxisVertexBufferView_;
  std::vector<D3D12_VERTEX_BUFFER_VIEW> mandelVertexBufferViews_;
  std::unique_ptr<VertexArray> vertexArray_;

  unsigned int controllerVertCount_;

  Matrix4 hmdPose_;
  Matrix4 eyePosLeft_;
  Matrix4 eyePosRight_;

  Matrix4 cameraPos_;
  float cameraMomentum_;
  Matrix4 magnification_;

  Matrix4 projectionCenter_;
  Matrix4 projectionLeft_;
  Matrix4 projectionRight_;

  struct VertexDataScene {
    Vector3 position;
    Vector2 texCoord;
  };

  struct VertexDataWindow {
    Vector2 position;
    Vector2 texCoord;

    VertexDataWindow(const Vector2& pos, const Vector2 tex)
        : position(pos), texCoord(tex) {}
  };

  struct FramebufferDesc {
    ComPtr<ID3D12Resource> texture;
    CD3DX12_CPU_DESCRIPTOR_HANDLE renderTargetViewHandle;
    ComPtr<ID3D12Resource> depthStencil;
    CD3DX12_CPU_DESCRIPTOR_HANDLE depthStencilViewHandle;
  };
  FramebufferDesc leftEyeDesc_;
  FramebufferDesc rightEyeDesc_;

  bool CreateFrameBuffer(int width, int height,
                         FramebufferDesc& framebufferDesc, RTVIndex rtvIndex);

  uint32_t renderWidth_;
  uint32_t renderHeight_;

  std::vector<RenderModel*> renderModels_;
  RenderModel* trackedDeviceToRenderModel_[vr::k_unMaxTrackedDeviceCount];
};