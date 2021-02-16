#include "MainApplication.h"

#include <mutex>

#include <windows.h>
#include <wrl.h>

#include <SDL.h>
#include <SDL_syswm.h>

#include <D3Dcompiler.h>
#include <d3dx12.h>
#include <d3d12.h>

#include <pathtools.h>

#include "ExtendList.h"
#include "RenderingConfig.h"
#include "QuadTree.h"
#include "VertexArray.h"
#include "RenderModel.h"
#include "SystemTime.h"

using Microsoft::WRL::ComPtr;

void ThreadSleep(unsigned long nMilliseconds) { ::Sleep(nMilliseconds); }

// Constructor
MainApplication::MainApplication(int argc, char* argv[])
    : companionWindow_(NULL),
      companionWindowWidth_(640),
      companionWindowHeight_(320),
      hmd_(NULL),
      ivrRenderModels_(NULL),
      debugD3D12_(false),
      verbose_(false),
      perf_(false),
      vBlank_(false),
      msaaSampleCount_(4),
      superSampleScale_(1.0f),
      trackedControllerCount_(0),
      trackedControllerCountLast_(-1),
      validPoseCount_(0),
      validPoseCountLast_(-1),
      poseClasses_(""),
      showMandelPlane_(true),
      frameIndex_(0),
      fenceEvent_(NULL),
      rtvDescriptorSize_(0),
      cbvSrvDescriptorSize_(0),
      dsvDescriptorSize_(0),
      maxLevel_(8),
      isShuttingDown_(false) {
  memset(sceneConstantBufferData_, 0, sizeof(sceneConstantBufferData_));

  for (int i = 1; i < argc; i++) {
    if (!_stricmp(argv[i], "-dxdebug")) {
      debugD3D12_ = true;
    } else if (!_stricmp(argv[i], "-verbose")) {
      verbose_ = true;
    } else if (!_stricmp(argv[i], "-novblank")) {
      vBlank_ = false;
    } else if (!_stricmp(argv[i], "-msaa") && (argc > i + 1) &&
               (*argv[i + 1] != '-')) {
      msaaSampleCount_ = atoi(argv[i + 1]);
      i++;
    } else if (!_stricmp(argv[i], "-supersample") && (argc > i + 1) &&
               (*argv[i + 1] != '-')) {
      superSampleScale_ = (float)atof(argv[i + 1]);
      i++;
    } else if (!_stricmp(argv[i], "-noprintf")) {
      g_bPrintf = false;
    } 
  }
  // other initialization tasks are done in BInit
  memset(devClassChar_, 0, sizeof(devClassChar_));
  memset(&controllerState_, 0, sizeof(controllerState_));

  UpdateLodLevels();
};

// Destructor
MainApplication::~MainApplication() {
  // work is done in Shutdown()
  dprintf("Shutdown!\n");
}

// Helper to get a string from a tracked device property and turn it into a
// std::string
std::string GetTrackedDeviceString(vr::IVRSystem* hmd,
                                   vr::TrackedDeviceIndex_t deviceIndex,
                                   vr::TrackedDeviceProperty prop,
                                   vr::TrackedPropertyError* error = NULL) {
  uint32_t requiredBufferLen =
      hmd->GetStringTrackedDeviceProperty(deviceIndex, prop, NULL, 0, error);
  if (requiredBufferLen == 0) return "";

  char* buffer = new char[requiredBufferLen];
  requiredBufferLen = hmd->GetStringTrackedDeviceProperty(
      deviceIndex, prop, buffer, requiredBufferLen, error);
  std::string result = buffer;
  delete[] buffer;
  return result;
}

bool MainApplication::BInit() {
  SystemTime::Initialize();

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
    dprintf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__,
            SDL_GetError());
    return false;
  }

  // Loading the SteamVR Runtime
  vr::EVRInitError error = vr::VRInitError_None;
  hmd_ = vr::VR_Init(&error, vr::VRApplication_Scene);

  if (error != vr::VRInitError_None) {
    hmd_ = NULL;
    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s",
              vr::VR_GetVRInitErrorAsEnglishDescription(error));
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
    return false;
  }

  ivrRenderModels_ = (vr::IVRRenderModels*)vr::VR_GetGenericInterface(
      vr::IVRRenderModels_Version, &error);
  if (!ivrRenderModels_) {
    hmd_ = NULL;
    vr::VR_Shutdown();

    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s",
              vr::VR_GetVRInitErrorAsEnglishDescription(error));
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
    return false;
  }

  int windowPosX = 700;
  int windowPosY = 100;
  Uint32 windowFlags = SDL_WINDOW_SHOWN;

  companionWindow_ = SDL_CreateWindow("MandelVR", windowPosX, windowPosY,
                                      companionWindowWidth_,
                                      companionWindowHeight_, windowFlags);
  if (companionWindow_ == NULL) {
    dprintf("%s - Window could not be created! SDL Error: %s\n", __FUNCTION__,
            SDL_GetError());
    return false;
  }

  driver_ = "No Driver";
  display_ = "No Display";

  driver_ = GetTrackedDeviceString(hmd_, vr::k_unTrackedDeviceIndex_Hmd,
                                   vr::Prop_TrackingSystemName_String);
  display_ = GetTrackedDeviceString(hmd_, vr::k_unTrackedDeviceIndex_Hmd,
                                    vr::Prop_SerialNumber_String);

  std::string windowTitle = "MandelVR - " + driver_ + " " + display_;
  SDL_SetWindowTitle(companionWindow_, windowTitle.c_str());

  // Mandel config
  mandelResolution_ = 500;

  nearClip_ = 0.1f;
  farClip_ = 30.0f;

  vertCount_ = 0;

  SYSTEM_INFO systemInfo;
  GetSystemInfo(&systemInfo);
  int numberOfCores = systemInfo.dwNumberOfProcessors;
  if (numberOfCores >= 4) {
      // Let's make sure we don't starve the SteamVR processes
    numberOfCores -= 2;
  }
  dprintf("Number of cores used for threadpool: %d\n", numberOfCores);

  threadPool_.reset(new thread_pool(numberOfCores));

  if (!BInitD3D12()) {
    dprintf("%s - Unable to initialize D3D12!\n", __FUNCTION__);
    return false;
  }

  if (!BInitCompositor()) {
    dprintf("%s - Failed to initialize VR Compositor!\n", __FUNCTION__);
    return false;
  }

  return true;
}

// Initialize DX12. Returns true if DX12 has been successfully
// initialized, false if shaders could not be created.
// If failure occurred in a module other than shaders, the function
// may return true or throw an error.
bool MainApplication::BInitD3D12() {
  UINT dxgiFactoryFlags = 0;

  // Debug layers if -dxdebug is specified
  if (debugD3D12_) {
    ComPtr<ID3D12Debug> pDebugController;
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&pDebugController)))) {
      pDebugController->EnableDebugLayer();
      dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
    }
  }

  ComPtr<IDXGIFactory4> factory;
  if (FAILED(CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&factory)))) {
    dprintf("CreateDXGIFactory2 failed.\n");
    return false;
  }

  // Query OpenVR for the output adapter index
  int32_t adapterIndex = 0;
  hmd_->GetDXGIOutputInfo(&adapterIndex);

  ComPtr<IDXGIAdapter1> adapter;
  if (FAILED(factory->EnumAdapters1(adapterIndex, &adapter))) {
    dprintf("Error enumerating DXGI adapter.\n");
  }
  DXGI_ADAPTER_DESC1 adapterDesc;
  adapter->GetDesc1(&adapterDesc);

  if (FAILED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0,
                               IID_PPV_ARGS(&device_)))) {
    dprintf("Failed to create D3D12 device with D3D12CreateDevice.\n");
    return false;
  }

  // Create the command queue
  D3D12_COMMAND_QUEUE_DESC queueDesc = {};
  queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
  queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
  if (FAILED(device_->CreateCommandQueue(&queueDesc,
                                         IID_PPV_ARGS(&commandQueue_)))) {
    printf("Failed to create D3D12 command queue.\n");
    return false;
  }

  // Create the swapchain
  DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
  swapChainDesc.BufferCount = kFrameCount;
  swapChainDesc.Width = companionWindowWidth_;
  swapChainDesc.Height = companionWindowHeight_;
  swapChainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
  swapChainDesc.SampleDesc.Count = 1;

  // Determine the HWND from SDL
  struct SDL_SysWMinfo wmInfo;
  SDL_VERSION(&wmInfo.version);
  SDL_GetWindowWMInfo(companionWindow_, &wmInfo);
  HWND hWnd = wmInfo.info.win.window;

  ComPtr<IDXGISwapChain1> swapChain;
  if (FAILED(factory->CreateSwapChainForHwnd(commandQueue_.Get(), hWnd,
                                             &swapChainDesc, nullptr, nullptr,
                                             &swapChain))) {
    dprintf("Failed to create DXGI swapchain.\n");
    return false;
  }

  factory->MakeWindowAssociation(hWnd, DXGI_MWA_NO_ALT_ENTER);
  swapChain.As(&swapChain_);
  frameIndex_ = swapChain_->GetCurrentBackBufferIndex();

  // Create descriptor heaps
  {
    rtvDescriptorSize_ = device_->GetDescriptorHandleIncrementSize(
        D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    dsvDescriptorSize_ = device_->GetDescriptorHandleIncrementSize(
        D3D12_DESCRIPTOR_HEAP_TYPE_DSV);
    cbvSrvDescriptorSize_ = device_->GetDescriptorHandleIncrementSize(
        D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
    rtvHeapDesc.NumDescriptors = NUM_RTVS;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    device_->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&rtvHeap_));

    D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
    rtvHeapDesc.NumDescriptors = NUM_RTVS;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    device_->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&dsvHeap_));

    D3D12_DESCRIPTOR_HEAP_DESC cbvSrvHeapDesc = {};
    cbvSrvHeapDesc.NumDescriptors = NUM_SRV_CBVS;
    cbvSrvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    cbvSrvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    device_->CreateDescriptorHeap(&cbvSrvHeapDesc, IID_PPV_ARGS(&cbvSrvHeap_));
  }

  // Create per-frame resources
  for (int frame = 0; frame < kFrameCount; frame++) {
    if (FAILED(device_->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_DIRECT,
            IID_PPV_ARGS(&commandAllocators_[frame])))) {
      dprintf("Failed to create command allocators.\n");
      return false;
    }

    // Create swapchain render targets
    swapChain_->GetBuffer(frame, IID_PPV_ARGS(&swapChainRenderTarget_[frame]));

    // Create swapchain render target views
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(
        rtvHeap_->GetCPUDescriptorHandleForHeapStart());
    rtvHandle.Offset(RTV_SWAPCHAIN0 + frame, rtvDescriptorSize_);
    device_->CreateRenderTargetView(swapChainRenderTarget_[frame].Get(),
                                    nullptr, rtvHandle);
  }

  // Create projection matrix constant buffer
  {
    device_->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(1024 * 64),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&sceneConstantBuffer_));

    // Keep as persistently mapped buffer, store left eye in first 256 bytes,
    // right eye in second
    UINT8* buffer;
    CD3DX12_RANGE readRange(0, 0);
    sceneConstantBuffer_->Map(0, &readRange, reinterpret_cast<void**>(&buffer));
    // Left eye to first 256 bytes, right eye to second 256 bytes
    sceneConstantBufferData_[0] = buffer;
    sceneConstantBufferData_[1] = buffer + 256;

    // Left eye CBV
    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvLeftEyeHandle(
        cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
    cbvLeftEyeHandle.Offset(CBV_LEFT_EYE, cbvSrvDescriptorSize_);
    D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
    cbvDesc.BufferLocation = sceneConstantBuffer_->GetGPUVirtualAddress();
    cbvDesc.SizeInBytes =
        512;  // (sizeof(Matrix4) + 255) & ~255; // Pad to 256 bytes
    device_->CreateConstantBufferView(&cbvDesc, cbvLeftEyeHandle);
    sceneConstantBufferView_[0] = cbvLeftEyeHandle;

    // Right eye CBV
    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvRightEyeHandle(
        cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
    cbvRightEyeHandle.Offset(CBV_RIGHT_EYE, cbvSrvDescriptorSize_);
    cbvDesc.BufferLocation += 256;
    device_->CreateConstantBufferView(&cbvDesc, cbvRightEyeHandle);
    sceneConstantBufferView_[1] = cbvRightEyeHandle;
  }

  // Create color lookup constant buffer
  {
    // Create CBR
    device_->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(1024 * 64),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&colorLookupConstantBuffer_));

    // Fill Data into buffer
    Vector4* buffer;
    CD3DX12_RANGE readRange(0, 0);
    colorLookupConstantBuffer_->Map(0, &readRange,
                                    reinterpret_cast<void**>(&buffer));

    int cycle_size = 16;
    int cycle_start = 0;
    for (int i = 0; i < 1024; i++) {
      int cycle = (i - cycle_start) % (3 * cycle_size);
      if (cycle < cycle_size) {
        buffer[i] = Vector4(0.0f, (float)cycle / cycle_size,
                            1.0f - ((float)cycle / cycle_size), 1.0f);
      } else if (cycle < (2 * cycle_size)) {
        cycle = cycle - cycle_size;
        buffer[i] = Vector4((float)cycle / cycle_size,
                            1.0f - ((float)cycle / cycle_size), 0.0f, 1.0f);
      } else {
        cycle = cycle - (2 * cycle_size);
        buffer[i] = Vector4(1.0f - ((float)cycle / cycle_size), 0.0f,
                            (float)cycle / cycle_size, 1.0f);
      }

      if (i - cycle_start >= 3 * cycle_size) {
        cycle_start += 3 * cycle_size;
        cycle_size += cycle_size / 2;
      }
    }
    for (int i = 511; i < 1024; i++) {
      buffer[i] = Vector4(0.1f, 0.1f, 0.2f, 1.0f);
    }

    colorLookupConstantBuffer_->Unmap(0, nullptr);

    // Create Left CBV
    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvLeftHandle(
        cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
    cbvLeftHandle.Offset(CBV_COLOR_LOOKUP_LEFT, cbvSrvDescriptorSize_);
    D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
    cbvDesc.BufferLocation = colorLookupConstantBuffer_->GetGPUVirtualAddress();
    cbvDesc.SizeInBytes =
        (1024 * sizeof(Vector4) + 255) & ~255;  // Pad to 256 bytes
    device_->CreateConstantBufferView(&cbvDesc, cbvLeftHandle);

    // Create Right CBV
    CD3DX12_CPU_DESCRIPTOR_HANDLE cbvRightHandle(
        cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
    cbvRightHandle.Offset(CBV_COLOR_LOOKUP_RIGHT, cbvSrvDescriptorSize_);
    device_->CreateConstantBufferView(&cbvDesc, cbvRightHandle);
  }

  // Create fence
  {
    memset(fenceValues_, 0, sizeof(fenceValues_));
    device_->CreateFence(fenceValues_[frameIndex_], D3D12_FENCE_FLAG_NONE,
                         IID_PPV_ARGS(&fence_));
    fenceValues_[frameIndex_]++;

    fenceEvent_ = CreateEvent(nullptr, FALSE, FALSE, nullptr);
  }

  if (!CreateAllShaders()) return false;

  // Create command list
  device_->CreateCommandList(
      0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAllocators_[frameIndex_].Get(),
      scenePipelineState_.Get(), IID_PPV_ARGS(&commandList_));

  SetupMandelScene();
  SetupCameras();
  SetupStereoRenderTargets();
  SetupCompanionWindow();
  SetupRenderModels();

  // Do any work that was queued up during loading
  commandList_->Close();
  ID3D12CommandList* ppCommandLists[] = {commandList_.Get()};
  commandQueue_->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);

  // Wait for it to finish
  commandQueue_->Signal(fence_.Get(), fenceValues_[frameIndex_]);
  fence_->SetEventOnCompletion(fenceValues_[frameIndex_], fenceEvent_);
  WaitForSingleObjectEx(fenceEvent_, INFINITE, FALSE);
  fenceValues_[frameIndex_]++;

  return true;
}

// Initialize Compositor. Returns true if the compositor was
// successfully initialized, false otherwise.
bool MainApplication::BInitCompositor() {
  vr::EVRInitError error = vr::VRInitError_None;

  if (!vr::VRCompositor()) {
    dprintf("Compositor initialization failed. See log file for details\n");
    return false;
  }

  return true;
}

void MainApplication::Shutdown() {
  { 
    std::unique_lock lock(shutdownMutex_);
    isShuttingDown_ = true;
  }

  threadPool_->wait_until_empty();

  if (hmd_) {
    vr::VR_Shutdown();
    hmd_ = NULL;
  }

  for (std::vector<RenderModel*>::iterator i = renderModels_.begin();
       i != renderModels_.end(); i++) {
    delete (*i);
  }
  renderModels_.clear();

  if (companionWindow_) {
    SDL_DestroyWindow(companionWindow_);
    companionWindow_ = NULL;
  }

  SDL_Quit();
}

bool MainApplication::IsShuttingDown() {
  bool result;

  {
    std::unique_lock lock(shutdownMutex_);
    result = isShuttingDown_;
  }

  return result;
}

bool MainApplication::HandleInput() {
  SDL_Event sdlEvent;
  bool result = false;

  while (SDL_PollEvent(&sdlEvent) != 0) {
    if (sdlEvent.type == SDL_QUIT) {
      result = true;
    } else if (sdlEvent.type == SDL_KEYDOWN) {
      if (sdlEvent.key.keysym.sym == SDLK_ESCAPE ||
          sdlEvent.key.keysym.sym == SDLK_q) {
        result = true;
      }
    }
  }

  // Process SteamVR events
  vr::VREvent_t event;
  while (hmd_->PollNextEvent(&event, sizeof(event))) {
    ProcessVREvent(event);
  }

  // Process SteamVR controller state
  for (vr::TrackedDeviceIndex_t deviceIndex = 0;
       deviceIndex < vr::k_unMaxTrackedDeviceCount; deviceIndex++) {
    vr::VRControllerState_t state;
    vr::VRControllerState_t& prev_state = controllerState_[deviceIndex];
    if (hmd_->GetControllerState(deviceIndex, &state, sizeof(state))) {
      const float kZoomInFactor = 0.9f;
      const float kZoomOutFactor = 1.0f / kZoomInFactor;

      if ((state.ulButtonPressed & 0x02) != 0) {
        // Zoom in.
        Vector4 oldCameraOffset = cameraPos_ * Vector4(0.0f, 0.0f, 0.0f, 1.0);
        Vector4 hmdOffset =
            ConvertSteamVRMatrixToMatrix4(
                trackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd]
                    .mDeviceToAbsoluteTracking) *
            Vector4(0.0f, 0.0f, 0.0f, 1.0);
        Vector4 newCameraOffset =
            kZoomOutFactor * (oldCameraOffset + hmdOffset) - hmdOffset;
        cameraPos_ = Matrix4().translate(newCameraOffset.x, newCameraOffset.y,
                                         newCameraOffset.z);

        magnification_.scale(kZoomInFactor, kZoomInFactor, kZoomInFactor);

        dprintf("magnification up to: %f\n", 1.0 / *magnification_.get());

        // Increase max level
        maxLevel_ =
            8 + static_cast<int>(std::floor(-std::log2(*magnification_.get())));
        maxLevel_ = min(maxLevel_, 127);

        UpdateLodLevels();
      }
      if ((state.ulButtonPressed & 0x04) != 0) {
        // Zoom out.
        Vector4 oldCameraOffset = cameraPos_ * Vector4(0.0f, 0.0f, 0.0f, 1.0);
        Vector4 hmdOffset =
            ConvertSteamVRMatrixToMatrix4(
                trackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd]
                    .mDeviceToAbsoluteTracking) *
            Vector4(0.0f, 0.0f, 0.0f, 1.0);
        Vector4 newCameraOffset =
            kZoomInFactor * (oldCameraOffset + hmdOffset) - hmdOffset;
        cameraPos_ = Matrix4().translate(newCameraOffset.x, newCameraOffset.y,
                                         newCameraOffset.z);

        magnification_.scale(kZoomOutFactor, kZoomOutFactor, kZoomOutFactor);

        dprintf("magnification down to: %f\n", 1.0 / *magnification_.get());

        // Decrease max level
        maxLevel_ =
            8 + static_cast<int>(std::floor(-std::log2(*magnification_.get())));
        maxLevel_ = min(maxLevel_, 127);

        UpdateLodLevels();
      }
      showTrackedDevice_[deviceIndex] = state.ulButtonPressed == 0;
    }

    // Move camera
    if (fabs(state.rAxis[0].x) > 0.001 || fabs(state.rAxis[0].y) > 0.001) {
      if (sqrt(state.rAxis[0].x * state.rAxis[0].x +
               state.rAxis[0].y * state.rAxis[0].y) > 0.95) {
        cameraMomentum_ *= 1.05;
      } else {
        cameraMomentum_ = 0.05f;
      }

      const Matrix4& mat = devicePose_[deviceIndex];

      Vector4 x = mat * Vector4(cameraMomentum_, 0, 0, 0);
      x *= state.rAxis[0].x;
      Vector4 y = mat * Vector4(0, 0, -cameraMomentum_, 0);
      y *= state.rAxis[0].y;
      Vector4 dir = x + y;

      Matrix4 dir_mat;
      dir_mat.translate(dir.x, dir.y, dir.z);
      cameraPos_ *= dir_mat;
    }

    controllerState_[deviceIndex] = state;
  }

  return result;
}

void MainApplication::RunMainLoop() {
  bool quit = false;

  SDL_StartTextInput();
  SDL_ShowCursor(SDL_DISABLE);

  int frameCount = 0;
  int64_t lastTick = SystemTime::GetCurrentTick();

  while (!quit) {
    quit = HandleInput();

    RenderFrame();

    frameCount++;
    int64_t current_tick = SystemTime::GetCurrentTick();
    double duration = SystemTime::TimeBetweenTicks(lastTick, current_tick);
    if (duration >= 1.0) {
      double frame_ms = 1000.0 / (double)frameCount;
      dprintf("%d fps, %.2lf ms, %d els\n", frameCount, frame_ms,
              extendList_.Size());

      lastTick = current_tick;
      frameCount = 0;
    }
  }

  SDL_StopTextInput();
}

// Processes a single VR event
void MainApplication::ProcessVREvent(const vr::VREvent_t& event) {
  switch (event.eventType) {
    case vr::VREvent_TrackedDeviceActivated: {
      SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
      dprintf("Device %u attached. Setting up render model.\n",
              event.trackedDeviceIndex);
    } break;
    case vr::VREvent_TrackedDeviceDeactivated: {
      dprintf("Device %u detached.\n", event.trackedDeviceIndex);
    } break;
    case vr::VREvent_TrackedDeviceUpdated: {
      dprintf("Device %u updated.\n", event.trackedDeviceIndex);
    } break;
  }
}

void MainApplication::RenderFrame() {
  if (hmd_) {
    commandAllocators_[frameIndex_]->Reset();

    commandList_->Reset(commandAllocators_[frameIndex_].Get(),
                        scenePipelineState_.Get());
    commandList_->SetGraphicsRootSignature(rootSignature_.Get());

    ID3D12DescriptorHeap* ppHeaps[] = {cbvSrvHeap_.Get()};
    commandList_->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);

    UpdateControllerAxes();
    RenderStereoTargets();
    RenderCompanionWindow();

    commandList_->Close();

    // Execute the command list.
    ID3D12CommandList* ppCommandLists[] = {commandList_.Get()};
    commandQueue_->ExecuteCommandLists(_countof(ppCommandLists),
                                       ppCommandLists);

    vr::VRTextureBounds_t bounds;
    bounds.uMin = 0.0f;
    bounds.uMax = 1.0f;
    bounds.vMin = 0.0f;
    bounds.vMax = 1.0f;

    vr::D3D12TextureData_t d3d12LeftEyeTexture = {leftEyeDesc_.texture.Get(),
                                                  commandQueue_.Get(), 0};
    vr::Texture_t leftEyeTexture = {(void*)&d3d12LeftEyeTexture,
                                    vr::TextureType_DirectX12,
                                    vr::ColorSpace_Gamma};
    vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture, &bounds,
                               vr::Submit_Default);

    vr::D3D12TextureData_t d3d12RightEyeTexture = {rightEyeDesc_.texture.Get(),
                                                   commandQueue_.Get(), 0};
    vr::Texture_t rightEyeTexture = {(void*)&d3d12RightEyeTexture,
                                     vr::TextureType_DirectX12,
                                     vr::ColorSpace_Gamma};
    vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture, &bounds,
                               vr::Submit_Default);
  }

  // Present
  swapChain_->Present(0, 0);

  // Maybe commit more vertices?
  vertexArray_->MaybeFlushToGpu();

  // Wait for completion
  {
    const UINT64 currentFenceValue = fenceValues_[frameIndex_];
    commandQueue_->Signal(fence_.Get(), currentFenceValue);

    frameIndex_ = swapChain_->GetCurrentBackBufferIndex();
    if (fence_->GetCompletedValue() < fenceValues_[frameIndex_]) {
      fence_->SetEventOnCompletion(fenceValues_[frameIndex_], fenceEvent_);
      WaitForSingleObjectEx(fenceEvent_, INFINITE, FALSE);
    }

    fenceValues_[frameIndex_] = currentFenceValue + 1;
  }

  // Spew out the controller and pose count whenever they change.
  if (trackedControllerCount_ != trackedControllerCountLast_ ||
      validPoseCount_ != validPoseCountLast_) {
    validPoseCountLast_ = validPoseCount_;
    trackedControllerCountLast_ = trackedControllerCount_;

    dprintf("PoseCount:%d(%s) Controllers:%d\n", validPoseCount_,
            poseClasses_.c_str(), trackedControllerCount_);
  }

  UpdateHMDMatrixPose();
}

bool MainApplication::CreateAllShaders() {
  std::string executableDirectory =
      Path_StripFilename(Path_GetExecutablePath());

  // Root signature
  {
    D3D12_FEATURE_DATA_ROOT_SIGNATURE featureData = {};
    featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_1;
    if (FAILED(device_->CheckFeatureSupport(
            D3D12_FEATURE_ROOT_SIGNATURE, &featureData, sizeof(featureData)))) {
      featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_0;
    }

    CD3DX12_DESCRIPTOR_RANGE1 ranges[2];
    CD3DX12_ROOT_PARAMETER1 rootParameters[2];

    ranges[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_CBV, 3, 0, 0,
                   D3D12_DESCRIPTOR_RANGE_FLAG_DATA_STATIC);
    ranges[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0, 0);
    rootParameters[0].InitAsDescriptorTable(1, &ranges[0],
                                            D3D12_SHADER_VISIBILITY_ALL);
    rootParameters[1].InitAsDescriptorTable(1, &ranges[1],
                                            D3D12_SHADER_VISIBILITY_PIXEL);

    D3D12_ROOT_SIGNATURE_FLAGS rootSignatureFlags =
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS;

    D3D12_STATIC_SAMPLER_DESC sampler = {};
    sampler.Filter = D3D12_FILTER_MIN_MAG_MIP_POINT;
    sampler.AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
    sampler.ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER;
    sampler.MaxLOD = D3D12_FLOAT32_MAX;
    sampler.ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
    rootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters, 1,
                               &sampler, rootSignatureFlags);
    ComPtr<ID3DBlob> signature;
    ComPtr<ID3DBlob> error;
    D3DX12SerializeVersionedRootSignature(
        &rootSignatureDesc, featureData.HighestVersion, &signature, &error);
    device_->CreateRootSignature(0, signature->GetBufferPointer(),
                                 signature->GetBufferSize(),
                                 IID_PPV_ARGS(&rootSignature_));
  }

  // Mandel shader
  {
    ComPtr<ID3DBlob> vertexShader;
    ComPtr<ID3DBlob> pixelShader;
    UINT compileFlags = 0;

    std::string shaderPath =
        Path_MakeAbsolute("./shaders/mandel.hlsl", executableDirectory);
    std::wstring shaderPathW =
        std::wstring(shaderPath.begin(), shaderPath.end());
    ComPtr<ID3DBlob> error;
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "VSMain", "vs_5_0", compileFlags, 0,
                                  &vertexShader, &error))) {
      dprintf("Failed compiling vertex shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "PSMain", "ps_5_0", compileFlags, 0,
                                  &pixelShader, &error))) {
      dprintf("Failed compiling pixel shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }

    // Define the vertex input layout.
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
    };

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = rootSignature_.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FillMode =
        D3D12_FILL_MODE_SOLID;  // D3D12_FILL_MODE_WIREFRAME
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.RasterizerState.MultisampleEnable = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = msaaSampleCount_;
    psoDesc.SampleDesc.Quality = 0;
    if (FAILED(device_->CreateGraphicsPipelineState(
            &psoDesc, IID_PPV_ARGS(&mandelPipelineState_)))) {
      dprintf("Error creating D3D12 pipeline state.\n");
      return false;
    }
  }

  // Companion shader
  {
    ComPtr<ID3DBlob> vertexShader;
    ComPtr<ID3DBlob> pixelShader;
    UINT compileFlags = 0;

    std::string shaderPath =
        Path_MakeAbsolute("./shaders/companion.hlsl", executableDirectory);
    std::wstring shaderPathW =
        std::wstring(shaderPath.begin(), shaderPath.end());
    ComPtr<ID3DBlob> error;
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "VSMain", "vs_5_0", compileFlags, 0,
                                  &vertexShader, &error))) {
      dprintf("Failed compiling vertex shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "PSMain", "ps_5_0", compileFlags, 0,
                                  &pixelShader, &error))) {
      dprintf("Failed compiling pixel shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }

    // Define the vertex input layout.
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 0,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 8,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
    };

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = rootSignature_.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState.DepthEnable = FALSE;
    psoDesc.DepthStencilState.StencilEnable = FALSE;
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.SampleDesc.Count = 1;
    if (FAILED(device_->CreateGraphicsPipelineState(
            &psoDesc, IID_PPV_ARGS(&companionPipelineState_)))) {
      dprintf("Error creating D3D12 pipeline state.\n");
      return false;
    }
  }

  // Axes shader
  {
    ComPtr<ID3DBlob> vertexShader;
    ComPtr<ID3DBlob> pixelShader;
    UINT compileFlags = 0;

    std::string shaderPath =
        Path_MakeAbsolute("./shaders/axes.hlsl", executableDirectory);
    std::wstring shaderPathW =
        std::wstring(shaderPath.begin(), shaderPath.end());
    ComPtr<ID3DBlob> error;
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "VSMain", "vs_5_0", compileFlags, 0,
                                  &vertexShader, &error))) {
      dprintf("Failed compiling vertex shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "PSMain", "ps_5_0", compileFlags, 0,
                                  &pixelShader, &error))) {
      dprintf("Failed compiling pixel shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }

    // Define the vertex input layout.
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
    };

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = rootSignature_.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.RasterizerState.MultisampleEnable = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = msaaSampleCount_;
    psoDesc.SampleDesc.Quality = 0;
    if (FAILED(device_->CreateGraphicsPipelineState(
            &psoDesc, IID_PPV_ARGS(&axesPipelineState_)))) {
      dprintf("Error creating D3D12 pipeline state.\n");
      return false;
    }
  }

  // Render Model shader
  {
    ComPtr<ID3DBlob> vertexShader;
    ComPtr<ID3DBlob> pixelShader;
    UINT compileFlags = 0;

    std::string shaderPath =
        Path_MakeAbsolute("./shaders/rendermodel.hlsl", executableDirectory);
    std::wstring shaderPathW =
        std::wstring(shaderPath.begin(), shaderPath.end());
    ComPtr<ID3DBlob> error;
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "VSMain", "vs_5_0", compileFlags, 0,
                                  &vertexShader, &error))) {
      dprintf("Failed compiling vertex shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }
    if (FAILED(D3DCompileFromFile(shaderPathW.c_str(), nullptr, nullptr,
                                  "PSMain", "ps_5_0", compileFlags, 0,
                                  &pixelShader, &error))) {
      dprintf("Failed compiling pixel shader '%s':\n%s\n", shaderPath.c_str(),
              (char*)error->GetBufferPointer());
      return false;
    }

    // Define the vertex input layout.
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
        {"TEXCOORD", 1, DXGI_FORMAT_R32G32_FLOAT, 0, 24,
         D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0},
    };

    // Describe and create the graphics pipeline state object (PSO).
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = {inputElementDescs, _countof(inputElementDescs)};
    psoDesc.pRootSignature = rootSignature_.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.FrontCounterClockwise = TRUE;
    psoDesc.RasterizerState.MultisampleEnable = TRUE;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = msaaSampleCount_;
    psoDesc.SampleDesc.Quality = 0;
    if (FAILED(device_->CreateGraphicsPipelineState(
            &psoDesc, IID_PPV_ARGS(&renderModelPipelineState_)))) {
      dprintf("Error creating D3D12 pipeline state.\n");
      return false;
    }
  }

  return true;
}

// Generate next level mipmap for an RGBA image
void MainApplication::GenMipMapRGBA(const UINT8* src, UINT8** dst,
                                     int srcWidth, int srcHeight,
                                     int* dstWidthOut, int* dstHeightOut) {
  *dstWidthOut = srcWidth / 2;
  if (*dstWidthOut <= 0) {
    *dstWidthOut = 1;
  }
  *dstHeightOut = srcHeight / 2;
  if (*dstHeightOut <= 0) {
    *dstHeightOut = 1;
  }

  *dst = new UINT8[4 * (*dstWidthOut) * (*dstHeightOut)];
  for (int y = 0; y < *dstHeightOut; y++) {
    for (int x = 0; x < *dstWidthOut; x++) {
      int nSrcIndex[4];
      float r = 0.0f;
      float g = 0.0f;
      float b = 0.0f;
      float a = 0.0f;

      nSrcIndex[0] = (((y * 2) * srcWidth) + (x * 2)) * 4;
      nSrcIndex[1] = (((y * 2) * srcWidth) + (x * 2 + 1)) * 4;
      nSrcIndex[2] = ((((y * 2) + 1) * srcWidth) + (x * 2)) * 4;
      nSrcIndex[3] = ((((y * 2) + 1) * srcWidth) + (x * 2 + 1)) * 4;

      // Sum all pixels
      for (int nSample = 0; nSample < 4; nSample++) {
        r += src[nSrcIndex[nSample]];
        g += src[nSrcIndex[nSample] + 1];
        b += src[nSrcIndex[nSample] + 2];
        a += src[nSrcIndex[nSample] + 3];
      }

      // Average results
      r /= 4.0;
      g /= 4.0;
      b /= 4.0;
      a /= 4.0;

      // Store resulting pixels
      (*dst)[(y * (*dstWidthOut) + x) * 4] = (UINT8)(r);
      (*dst)[(y * (*dstWidthOut) + x) * 4 + 1] = (UINT8)(g);
      (*dst)[(y * (*dstWidthOut) + x) * 4 + 2] = (UINT8)(b);
      (*dst)[(y * (*dstWidthOut) + x) * 4 + 3] = (UINT8)(a);
    }
  }
}


void MainApplication::SetupMandelScene() {
  if (!hmd_) return;

  quadTree_ = new QuadTree(nullptr, 0.0f, 0.0f, 0);

  vertexArray_.reset(new VertexArray(device_));
  vertexArray_->Init();

  // Create initial nodes in quadtree
  CpuTimer timer;
  timer.Start();
  {
    QuadTree* root = quadTree_;
    thread_pool* thread_pool = threadPool_.get();
    std::function<void()> workFun = [this, &thread_pool, root]() {
      root->Setup(3, thread_pool, vertexArray_.get());
    };
    threadPool_->enqueue_task(workFun);

    threadPool_->wait_until_empty();
  }
  timer.Stop();
  dprintf("Setup elapsed time: %f\n", timer.GetTime());

  // Add worker threads to extend quadtree on demand
  thread_pool* thread_pool = threadPool_.get();
  for (int i = 0; i < thread_pool->size(); i++) {
    std::function<void()> work_fun = [this]() {
      while (!IsShuttingDown()) {
        QuadTree* cell = extendList_.PopCell();
        // TODO Make extendList have the thread wait instead.
        if (cell == nullptr) {
          ThreadSleep(500);
          continue;
        }
        const std::vector<float>& vertices = cell->Compute();
        vertexArray_->AddVertices(cell, vertices);
      }
    };
    threadPool_->enqueue_task(work_fun);
  }

  mandelVertexBufferViews_.emplace_back();
  D3D12_VERTEX_BUFFER_VIEW& vertex_buffer_view =
      mandelVertexBufferViews_.back();
  vertexArray_->FlushToGpu();
}

void MainApplication::CreateFrustumPlanes(vr::Hmd_Eye eye,
                                           std::vector<Vector4>* planes,
                                           Vector3* outCameraPos) {
  Matrix4 cameraToWorld;
  Vector3 cameraPos;
  const float* eyeMat = nullptr;

  if (eye == vr::Eye_Left) {
    eyeMat = projectionLeft_.get();
    cameraToWorld =
        (eyePosLeft_ * /* m_matMagnification * */ hmdPose_).invert();
    Vector4 left_camera_v4 = cameraToWorld * Vector4(0.0f, 0.0f, 0.0f, 1.0f);
    cameraPos = Vector3(left_camera_v4.x, left_camera_v4.y, left_camera_v4.z);
  } else {
    eyeMat = projectionRight_.get();
    cameraToWorld =
        (eyePosRight_ * /* m_matMagnification * */ hmdPose_).invert();
    Vector4 right_camera_v4 = cameraToWorld * Vector4(0.0f, 0.0f, 0.0f, 1.0f);
    cameraPos =
        Vector3(right_camera_v4.x, right_camera_v4.y, right_camera_v4.z);
  }

  *outCameraPos = cameraPos;

  // Left Frustum Plane
  {
    Vector4 planeNormal =
        (cameraToWorld *
         Vector4(eyeMat[0 + 0 * 4], 0.0f, eyeMat[0 + 2 * 4] - 1.0f, 0.0f))
            .normalize();
    float w =
        -(Vector3(planeNormal.x, planeNormal.y, planeNormal.z).dot(cameraPos));
    planes->push_back(Vector4(planeNormal.x, planeNormal.y, planeNormal.z, w));
  }

  // Right Frustum Plane
  {
    Vector4 planeNormal =
        (cameraToWorld *
         Vector4(-eyeMat[0 + 0 * 4], 0.0f, -eyeMat[0 + 2 * 4] - 1.0f, 0.0f))
            .normalize();
    float w =
        -(Vector3(planeNormal.x, planeNormal.y, planeNormal.z).dot(cameraPos));
    planes->push_back(Vector4(planeNormal.x, planeNormal.y, planeNormal.z, w));
  }

  // Top Frustum Plane
  {
    Vector4 planeNormal =
        (cameraToWorld *
         Vector4(0.0f, -eyeMat[1 + 1 * 4], -eyeMat[1 + 2 * 4] - 1.0f, 0.0f))
            .normalize();
    float w =
        -(Vector3(planeNormal.x, planeNormal.y, planeNormal.z).dot(cameraPos));
    planes->push_back(Vector4(planeNormal.x, planeNormal.y, planeNormal.z, w));
  }

  // Bottom Frustum Plane
  {
    Vector4 planeNormal =
        (cameraToWorld *
         Vector4(0.0f, eyeMat[1 + 1 * 4], eyeMat[1 + 2 * 4] - 1.0f, 0.0f))
            .normalize();
    float w =
        -(Vector3(planeNormal.x, planeNormal.y, planeNormal.z).dot(cameraPos));
    planes->push_back(Vector4(planeNormal.x, planeNormal.y, planeNormal.z, w));
  }

  // No near and far clipping as it's not worth the hassle for now.
  planes->push_back(Vector4(0.0f, 0.0f, 0.0f, 10.0f));
  planes->push_back(Vector4(0.0f, 0.0f, 0.0f, 10.0f));

  return;
}

// Update the vertex data for the controllers as X/Y/Z lines
void MainApplication::UpdateControllerAxes() {
  // Don't attempt to update controllers if input is not available
  if (!hmd_->IsInputAvailable()) return;

  std::vector<float> verticesArray;

  controllerVertCount_ = 0;
  trackedControllerCount_ = 0;

  for (vr::TrackedDeviceIndex_t trackedDeviceIndex =
           vr::k_unTrackedDeviceIndex_Hmd + 1;
       trackedDeviceIndex < vr::k_unMaxTrackedDeviceCount;
       ++trackedDeviceIndex) {
    if (!hmd_->IsTrackedDeviceConnected(trackedDeviceIndex)) continue;

    if (hmd_->GetTrackedDeviceClass(trackedDeviceIndex) !=
        vr::TrackedDeviceClass_Controller)
      continue;

    trackedControllerCount_ += 1;

    if (!trackedDevicePose_[trackedDeviceIndex].bPoseIsValid) continue;

    const Matrix4& mat = devicePose_[trackedDeviceIndex];

    Vector4 center = mat * Vector4(0, 0, 0, 1);

    for (int i = 0; i < 3; ++i) {
      Vector3 color(0, 0, 0);
      Vector4 point(0, 0, 0, 1);
      point[i] += 0.05f;  // offset in X, Y, Z
      color[i] = 1.0;     // R, G, B
      point = mat * point;
      verticesArray.push_back(center.x);
      verticesArray.push_back(center.y);
      verticesArray.push_back(center.z);

      verticesArray.push_back(color.x);
      verticesArray.push_back(color.y);
      verticesArray.push_back(color.z);

      verticesArray.push_back(point.x);
      verticesArray.push_back(point.y);
      verticesArray.push_back(point.z);

      verticesArray.push_back(color.x);
      verticesArray.push_back(color.y);
      verticesArray.push_back(color.z);

      controllerVertCount_ += 2;
    }

    Vector4 start = mat * Vector4(0, 0, -0.02f, 1);
    Vector4 end = mat * Vector4(0, 0, -39.f, 1);
    Vector3 color(.92f, .92f, .71f);

    verticesArray.push_back(start.x);
    verticesArray.push_back(start.y);
    verticesArray.push_back(start.z);
    verticesArray.push_back(color.x);
    verticesArray.push_back(color.y);
    verticesArray.push_back(color.z);

    verticesArray.push_back(end.x);
    verticesArray.push_back(end.y);
    verticesArray.push_back(end.z);
    verticesArray.push_back(color.x);
    verticesArray.push_back(color.y);
    verticesArray.push_back(color.z);
    controllerVertCount_ += 2;
  }

  // Setup the VB the first time through.
  if (controllerAxisVertexBuffer_ == nullptr && verticesArray.size() > 0) {
    // Make big enough to hold up to the max number
    size_t arraySize = sizeof(float) * verticesArray.size();
    arraySize *= vr::k_unMaxTrackedDeviceCount;

    device_->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(arraySize),
        D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
        IID_PPV_ARGS(&controllerAxisVertexBuffer_));

    controllerAxisVertexBufferView_.BufferLocation =
        controllerAxisVertexBuffer_->GetGPUVirtualAddress();
    controllerAxisVertexBufferView_.StrideInBytes = sizeof(float) * 6;
    controllerAxisVertexBufferView_.SizeInBytes =
        sizeof(float) * verticesArray.size();
  }

  // Update the VB data
  if (controllerAxisVertexBuffer_ && verticesArray.size() > 0) {
    UINT8* pMappedBuffer;
    CD3DX12_RANGE readRange(0, 0);
    controllerAxisVertexBuffer_->Map(0, &readRange,
                                     reinterpret_cast<void**>(&pMappedBuffer));
    memcpy(pMappedBuffer, &verticesArray[0],
           sizeof(float) * verticesArray.size());
    controllerAxisVertexBuffer_->Unmap(0, nullptr);
  }
}

void MainApplication::SetupCameras() {
  cameraPos_.translate(0.0f, 0.0f, 0.0f);
  magnification_.scale(1.0f, 1.0f, 1.0f);

  projectionLeft_ = GetHMDMatrixProjectionEye(vr::Eye_Left);
  projectionRight_ = GetHMDMatrixProjectionEye(vr::Eye_Right);
  eyePosLeft_ = GetHMDMatrixPoseEye(vr::Eye_Left);
  eyePosRight_ = GetHMDMatrixPoseEye(vr::Eye_Right);
}

// Creates a frame buffer. Returns true if the buffer was set up.
// Returns false if the setup failed.
bool MainApplication::CreateFrameBuffer(int width, int height,
                                         FramebufferDesc& framebufferDesc,
                                         RTVIndex rtvIndex) {
  D3D12_RESOURCE_DESC textureDesc = {};
  textureDesc.MipLevels = 1;
  textureDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
  textureDesc.Width = width;
  textureDesc.Height = height;
  textureDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;
  textureDesc.DepthOrArraySize = 1;
  textureDesc.SampleDesc.Count = msaaSampleCount_;
  textureDesc.SampleDesc.Quality = 0;
  textureDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;

  const float clearColor[] = {0.0f, 0.0f, 0.0f, 1.0f};

  // Create color target
  device_->CreateCommittedResource(
      &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT), D3D12_HEAP_FLAG_NONE,
      &textureDesc, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
      &CD3DX12_CLEAR_VALUE(DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, clearColor),
      IID_PPV_ARGS(&framebufferDesc.texture));

  CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(
      rtvHeap_->GetCPUDescriptorHandleForHeapStart());
  rtvHandle.Offset(rtvIndex, rtvDescriptorSize_);
  device_->CreateRenderTargetView(framebufferDesc.texture.Get(), nullptr,
                                  rtvHandle);
  framebufferDesc.renderTargetViewHandle = rtvHandle;

  // Create shader resource view
  CD3DX12_CPU_DESCRIPTOR_HANDLE srvHandle(
      cbvSrvHeap_->GetCPUDescriptorHandleForHeapStart());
  srvHandle.Offset(SRV_LEFT_EYE + rtvIndex, cbvSrvDescriptorSize_);
  device_->CreateShaderResourceView(framebufferDesc.texture.Get(), nullptr,
                                    srvHandle);

  // Create depth
  D3D12_RESOURCE_DESC depthDesc = textureDesc;
  depthDesc.Format = DXGI_FORMAT_D32_FLOAT;
  depthDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
  device_->CreateCommittedResource(
      &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT), D3D12_HEAP_FLAG_NONE,
      &depthDesc, D3D12_RESOURCE_STATE_DEPTH_WRITE,
      &CD3DX12_CLEAR_VALUE(DXGI_FORMAT_D32_FLOAT, 1.0f, 0),
      IID_PPV_ARGS(&framebufferDesc.depthStencil));

  CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(
      dsvHeap_->GetCPUDescriptorHandleForHeapStart());
  dsvHandle.Offset(rtvIndex, dsvDescriptorSize_);
  device_->CreateDepthStencilView(framebufferDesc.depthStencil.Get(), nullptr,
                                  dsvHandle);
  framebufferDesc.depthStencilViewHandle = dsvHandle;
  return true;
}

bool MainApplication::SetupStereoRenderTargets() {
  if (!hmd_) return false;

  hmd_->GetRecommendedRenderTargetSize(&renderWidth_, &renderHeight_);
  renderWidth_ = (uint32_t)(superSampleScale_ * (float)renderWidth_);
  renderHeight_ = (uint32_t)(superSampleScale_ * (float)renderHeight_);

  CreateFrameBuffer(renderWidth_, renderHeight_, leftEyeDesc_, RTV_LEFT_EYE);
  CreateFrameBuffer(renderWidth_, renderHeight_, rightEyeDesc_, RTV_RIGHT_EYE);
  return true;
}

void MainApplication::SetupCompanionWindow() {
  if (!hmd_) return;

  std::vector<VertexDataWindow> verts;

  // left eye verts
  verts.push_back(VertexDataWindow(Vector2(-1, -1), Vector2(0, 1)));
  verts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(1, 1)));
  verts.push_back(VertexDataWindow(Vector2(-1, 1), Vector2(0, 0)));
  verts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(1, 0)));

  // right eye verts
  verts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(0, 1)));
  verts.push_back(VertexDataWindow(Vector2(1, -1), Vector2(1, 1)));
  verts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(0, 0)));
  verts.push_back(VertexDataWindow(Vector2(1, 1), Vector2(1, 0)));

  device_->CreateCommittedResource(
      &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
      &CD3DX12_RESOURCE_DESC::Buffer(sizeof(VertexDataWindow) * verts.size()),
      D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
      IID_PPV_ARGS(&companionWindowVertexBuffer_));

  UINT8* mappedBuffer;
  CD3DX12_RANGE readRange(0, 0);
  companionWindowVertexBuffer_->Map(0, &readRange,
                                    reinterpret_cast<void**>(&mappedBuffer));
  memcpy(mappedBuffer, &verts[0], sizeof(VertexDataWindow) * verts.size());
  companionWindowVertexBuffer_->Unmap(0, nullptr);

  companionWindowVertexBufferView_.BufferLocation =
      companionWindowVertexBuffer_->GetGPUVirtualAddress();
  companionWindowVertexBufferView_.StrideInBytes = sizeof(VertexDataWindow);
  companionWindowVertexBufferView_.SizeInBytes =
      sizeof(VertexDataWindow) * verts.size();

  UINT16 indices[] = {0, 1, 3, 0, 3, 2, 4, 5, 7, 4, 7, 6};
  companionWindowIndexSize_ = _countof(indices);

  device_->CreateCommittedResource(
      &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD), D3D12_HEAP_FLAG_NONE,
      &CD3DX12_RESOURCE_DESC::Buffer(sizeof(indices)),
      D3D12_RESOURCE_STATE_GENERIC_READ, nullptr,
      IID_PPV_ARGS(&companionWindowIndexBuffer_));

  companionWindowIndexBuffer_->Map(0, &readRange,
                                   reinterpret_cast<void**>(&mappedBuffer));
  memcpy(mappedBuffer, &indices[0], sizeof(indices));
  companionWindowIndexBuffer_->Unmap(0, nullptr);

  companionWindowIndexBufferView_.BufferLocation =
      companionWindowIndexBuffer_->GetGPUVirtualAddress();
  companionWindowIndexBufferView_.Format = DXGI_FORMAT_R16_UINT;
  companionWindowIndexBufferView_.SizeInBytes = sizeof(indices);
}

void MainApplication::RenderStereoTargets() {
  D3D12_VIEWPORT viewport = {
      0.0f, 0.0f, (FLOAT)renderWidth_, (FLOAT)renderHeight_, 0.0f, 1.0f};
  D3D12_RECT scissor = {0, 0, (LONG)renderWidth_, (LONG)renderHeight_};

  commandList_->RSSetViewports(1, &viewport);
  commandList_->RSSetScissorRects(1, &scissor);

  //----------//
  // Left Eye //
  //----------//
  // Transition to RENDER_TARGET
  commandList_->ResourceBarrier(1,
                                &CD3DX12_RESOURCE_BARRIER::Transition(
                                    leftEyeDesc_.texture.Get(),
                                    D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                                    D3D12_RESOURCE_STATE_RENDER_TARGET));
  commandList_->OMSetRenderTargets(1, &leftEyeDesc_.renderTargetViewHandle,
                                   FALSE, &leftEyeDesc_.depthStencilViewHandle);

  const float clearColor[] = {0.0f, 0.0f, 0.0f, 1.0f};
  commandList_->ClearRenderTargetView(leftEyeDesc_.renderTargetViewHandle,
                                      clearColor, 0, nullptr);
  commandList_->ClearDepthStencilView(leftEyeDesc_.depthStencilViewHandle,
                                      D3D12_CLEAR_FLAG_DEPTH, 1.0, 0, 0,
                                      nullptr);

  RenderScene(vr::Eye_Left);

  // Transition to SHADER_RESOURCE to submit to SteamVR
  commandList_->ResourceBarrier(
      1, &CD3DX12_RESOURCE_BARRIER::Transition(
             leftEyeDesc_.texture.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET,
             D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE));

  //-----------//
  // Right Eye //
  //-----------//
  // Transition to RENDER_TARGET
  commandList_->ResourceBarrier(1,
                                &CD3DX12_RESOURCE_BARRIER::Transition(
                                    rightEyeDesc_.texture.Get(),
                                    D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
                                    D3D12_RESOURCE_STATE_RENDER_TARGET));
  commandList_->OMSetRenderTargets(1, &rightEyeDesc_.renderTargetViewHandle,
                                   FALSE,
                                   &rightEyeDesc_.depthStencilViewHandle);

  commandList_->ClearRenderTargetView(rightEyeDesc_.renderTargetViewHandle,
                                      clearColor, 0, nullptr);
  commandList_->ClearDepthStencilView(rightEyeDesc_.depthStencilViewHandle,
                                      D3D12_CLEAR_FLAG_DEPTH, 1.0, 0, 0,
                                      nullptr);

  RenderScene(vr::Eye_Right);

  // Transition to SHADER_RESOURCE to submit to SteamVR
  commandList_->ResourceBarrier(
      1, &CD3DX12_RESOURCE_BARRIER::Transition(
             rightEyeDesc_.texture.Get(), D3D12_RESOURCE_STATE_RENDER_TARGET,
             D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE));
}

// Renders a scene with respect to nEye.
void MainApplication::RenderScene(vr::Hmd_Eye eye) {
  if (showMandelPlane_) {
    // draw the mandel plane
    commandList_->SetPipelineState(mandelPipelineState_.Get());

    // Select the CBV (left or right eye)
    CD3DX12_GPU_DESCRIPTOR_HANDLE cbvHandle(
        cbvSrvHeap_->GetGPUDescriptorHandleForHeapStart());
    cbvHandle.Offset(eye, cbvSrvDescriptorSize_);
    commandList_->SetGraphicsRootDescriptorTable(0, cbvHandle);

    std::vector<Vector4> frustumPlanes;
    Vector3 cameraPos;
    CreateFrustumPlanes(eye, &frustumPlanes, &cameraPos);

    // Update the persistently mapped pointer to the CB data with the latest
    // matrix
    memcpy(sceneConstantBufferData_[eye],
           GetCurrentViewProjectionMatrix(eye).get(), sizeof(Matrix4));

    float minHeightFactor = *magnification_.get() / 10.0f;
    float heightFactor =
        log(max(-1.0, cameraPos.y) + 1.0f + minHeightFactor) + minHeightFactor;
    heightFactor = min(1.5, max(minHeightFactor, heightFactor));
    *((float*)(&sceneConstantBufferData_[eye][sizeof(Matrix4)])) = heightFactor;

    commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    commandList_->IASetVertexBuffers(0, 1, &mandelVertexBufferViews_[0]);

    RenderingConfig setup = RenderingConfig::Create(maxLevel_);
    setup.AddFrustumCulling(frustumPlanes, cameraPos);
    setup.EnableLod(lodLevels_);

    vertexArray_->StartRendering(commandList_);
    quadTree_->Render(setup, &extendList_, vertexArray_.get());
    vertexArray_->StopRendering();
  }

  bool isInputAvailable = hmd_->IsInputAvailable();

  if (isInputAvailable && controllerAxisVertexBuffer_) {
    // draw the controller axis lines
    commandList_->SetPipelineState(axesPipelineState_.Get());

    commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
    commandList_->IASetVertexBuffers(0, 1, &controllerAxisVertexBufferView_);
    commandList_->DrawInstanced(controllerVertCount_, 1, 0, 0);
  }

  // ----- Render Model rendering -----
  commandList_->SetPipelineState(renderModelPipelineState_.Get());
  for (uint32_t trackedDeviceIndex = 0;
       trackedDeviceIndex < vr::k_unMaxTrackedDeviceCount;
       trackedDeviceIndex++) {
    if (!trackedDeviceToRenderModel_[trackedDeviceIndex] ||
        !showTrackedDevice_[trackedDeviceIndex])
      continue;

    const vr::TrackedDevicePose_t& pose =
        trackedDevicePose_[trackedDeviceIndex];
    if (!pose.bPoseIsValid) continue;

    if (!isInputAvailable && hmd_->GetTrackedDeviceClass(trackedDeviceIndex) ==
                                 vr::TrackedDeviceClass_Controller)
      continue;

    const Matrix4& matDeviceToTracking = devicePose_[trackedDeviceIndex];
    Matrix4 matMVP = GetCurrentViewProjectionMatrix(eye) * matDeviceToTracking;

    trackedDeviceToRenderModel_[trackedDeviceIndex]->Draw(
        eye, commandList_.Get(), cbvSrvDescriptorSize_, matMVP);
  }
}

void MainApplication::RenderCompanionWindow() {
  commandList_->SetPipelineState(companionPipelineState_.Get());

  // Transition swapchain image to RENDER_TARGET
  commandList_->ResourceBarrier(
      1, &CD3DX12_RESOURCE_BARRIER::Transition(
             swapChainRenderTarget_[frameIndex_].Get(),
             D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

  // Bind current swapchain image
  CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(
      rtvHeap_->GetCPUDescriptorHandleForHeapStart());
  rtvHandle.Offset(RTV_SWAPCHAIN0 + frameIndex_, rtvDescriptorSize_);
  commandList_->OMSetRenderTargets(1, &rtvHandle, 0, nullptr);

  D3D12_VIEWPORT viewport = {
      0.0f, 0.0f, (FLOAT)companionWindowWidth_, (FLOAT)companionWindowHeight_,
      0.0f, 1.0f};
  D3D12_RECT scissor = {0, 0, (LONG)companionWindowWidth_,
                        (LONG)companionWindowHeight_};

  commandList_->RSSetViewports(1, &viewport);
  commandList_->RSSetScissorRects(1, &scissor);

  // render left eye (first half of index array)
  CD3DX12_GPU_DESCRIPTOR_HANDLE srvHandleLeftEye(
      cbvSrvHeap_->GetGPUDescriptorHandleForHeapStart());
  srvHandleLeftEye.Offset(SRV_LEFT_EYE, cbvSrvDescriptorSize_);
  commandList_->SetGraphicsRootDescriptorTable(1, srvHandleLeftEye);

  commandList_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  commandList_->IASetVertexBuffers(0, 1, &companionWindowVertexBufferView_);
  commandList_->IASetIndexBuffer(&companionWindowIndexBufferView_);
  commandList_->DrawIndexedInstanced(companionWindowIndexSize_ / 2, 1, 0, 0, 0);

  // render right eye (second half of index array)
  CD3DX12_GPU_DESCRIPTOR_HANDLE srvHandleRightEye(
      cbvSrvHeap_->GetGPUDescriptorHandleForHeapStart());
  srvHandleRightEye.Offset(SRV_RIGHT_EYE, cbvSrvDescriptorSize_);
  commandList_->SetGraphicsRootDescriptorTable(1, srvHandleRightEye);
  commandList_->DrawIndexedInstanced(companionWindowIndexSize_ / 2, 1,
                                     (companionWindowIndexSize_ / 2), 0, 0);

  // Transition swapchain image to PRESENT
  commandList_->ResourceBarrier(
      1, &CD3DX12_RESOURCE_BARRIER::Transition(
             swapChainRenderTarget_[frameIndex_].Get(),
             D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));
}

// Gets a Matrix Projection Eye with respect to eye.
Matrix4 MainApplication::GetHMDMatrixProjectionEye(vr::Hmd_Eye eye) {
  if (!hmd_) return Matrix4();

  vr::HmdMatrix44_t mat = hmd_->GetProjectionMatrix(eye, nearClip_, farClip_);

  return Matrix4(mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
                 mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
                 mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
                 mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]);
}

// Gets an HMDMatrixPoseEye with respect to nEye.
Matrix4 MainApplication::GetHMDMatrixPoseEye(vr::Hmd_Eye eye) {
  if (!hmd_) return Matrix4();

  vr::HmdMatrix34_t matEyeRight = hmd_->GetEyeToHeadTransform(eye);
  Matrix4 matrixObj(
      matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
      matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
      matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
      matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f);

  return matrixObj.invert();
}

// Gets a Current View Projection Matrix with respect to eye,
// which may be an Eye_Left or an Eye_Right.
Matrix4 MainApplication::GetCurrentViewProjectionMatrix(vr::Hmd_Eye eye) {
  Matrix4 matMVP;
  if (eye == vr::Eye_Left) {
    matMVP = projectionLeft_ * eyePosLeft_ * hmdPose_;
  } else if (eye == vr::Eye_Right) {
    matMVP = projectionRight_ * eyePosRight_ * hmdPose_;
  }

  return matMVP;
}

void MainApplication::UpdateHMDMatrixPose() {
  if (!hmd_) return;

  vr::VRCompositor()->WaitGetPoses(trackedDevicePose_,
                                   vr::k_unMaxTrackedDeviceCount, NULL, 0);

  validPoseCount_ = 0;
  poseClasses_ = "";
  for (int deviceIndex = 0; deviceIndex < vr::k_unMaxTrackedDeviceCount;
       ++deviceIndex) {
    if (trackedDevicePose_[deviceIndex].bPoseIsValid) {
      validPoseCount_++;
      devicePose_[deviceIndex] =
          magnification_ * cameraPos_ *
          ConvertSteamVRMatrixToMatrix4(
              trackedDevicePose_[deviceIndex].mDeviceToAbsoluteTracking);
      if (devClassChar_[deviceIndex] == 0) {
        switch (hmd_->GetTrackedDeviceClass(deviceIndex)) {
          case vr::TrackedDeviceClass_Controller:
            devClassChar_[deviceIndex] = 'C';
            break;
          case vr::TrackedDeviceClass_HMD:
            devClassChar_[deviceIndex] = 'H';
            break;
          case vr::TrackedDeviceClass_Invalid:
            devClassChar_[deviceIndex] = 'I';
            break;
          case vr::TrackedDeviceClass_GenericTracker:
            devClassChar_[deviceIndex] = 'G';
            break;
          case vr::TrackedDeviceClass_TrackingReference:
            devClassChar_[deviceIndex] = 'T';
            break;
          default:
            devClassChar_[deviceIndex] = '?';
            break;
        }
      }
      poseClasses_ += devClassChar_[deviceIndex];
    }
  }

  if (trackedDevicePose_[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid) {
    hmdPose_ = devicePose_[vr::k_unTrackedDeviceIndex_Hmd];
    hmdPose_.invert();
  }
}

void MainApplication::UpdateLodLevels() {
  lodLevels_.clear();
  lodLevels_.push_back(10000.0f);
  float lod = 10.0f;
  for (int i = 1; i < kMaxNumLevels; i++) {
    lodLevels_.push_back(lod);
    lod = lod / 2.0;
  }
}

// Finds a render model we've already loaded or loads a new one
RenderModel* MainApplication::FindOrLoadRenderModel(
    vr::TrackedDeviceIndex_t trackedDeviceIndex, const char* renderModelName) {
  RenderModel* renderModel = NULL;
  for (std::vector<RenderModel*>::iterator i = renderModels_.begin();
       i != renderModels_.end(); i++) {
    if (!_stricmp((*i)->GetName().c_str(), renderModelName)) {
      renderModel = *i;
      break;
    }
  }

  // load the model if we didn't find one
  if (!renderModel) {
    vr::RenderModel_t* model;
    vr::EVRRenderModelError error;
    while (true) {
      error =
          vr::VRRenderModels()->LoadRenderModel_Async(renderModelName, &model);
      if (error != vr::VRRenderModelError_Loading) break;

      ThreadSleep(1);
    }

    if (error != vr::VRRenderModelError_None) {
      dprintf("Unable to load render model %s - %s\n", renderModelName,
              vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
      return NULL;  // move on to the next tracked device
    }

    vr::RenderModel_TextureMap_t* texture;
    while (true) {
      error = vr::VRRenderModels()->LoadTexture_Async(model->diffuseTextureId,
                                                      &texture);
      if (error != vr::VRRenderModelError_Loading) break;

      ThreadSleep(1);
    }

    if (error != vr::VRRenderModelError_None) {
      dprintf("Unable to load render texture id:%d for render model %s\n",
              model->diffuseTextureId, renderModelName);
      vr::VRRenderModels()->FreeRenderModel(model);
      return NULL;  // move on to the next tracked device
    }

    renderModel = new RenderModel(renderModelName);
    if (!renderModel->BInit(device_.Get(), commandList_.Get(),
                            cbvSrvHeap_.Get(), trackedDeviceIndex, *model,
                            *texture)) {
      dprintf("Unable to create D3D12 model from render model %s\n",
              renderModelName);
      delete renderModel;
      renderModel = NULL;
    } else {
      renderModels_.push_back(renderModel);
    }
    vr::VRRenderModels()->FreeRenderModel(model);
    vr::VRRenderModels()->FreeTexture(texture);
  }

  return renderModel;
}

// Create/destroy D3D12 a Render Model for a single tracked device
void MainApplication::SetupRenderModelForTrackedDevice(
    vr::TrackedDeviceIndex_t trackedDeviceIndex) {
  if (trackedDeviceIndex >= vr::k_unMaxTrackedDeviceCount) return;

  // try to find a model we've already set up
  std::string renderModelName = GetTrackedDeviceString(
      hmd_, trackedDeviceIndex, vr::Prop_RenderModelName_String);
  RenderModel* renderModel =
      FindOrLoadRenderModel(trackedDeviceIndex, renderModelName.c_str());
  if (!renderModel) {
    std::string trackingSystemName = GetTrackedDeviceString(
        hmd_, trackedDeviceIndex, vr::Prop_TrackingSystemName_String);
    dprintf("Unable to load render model for tracked device %d (%s.%s)",
            trackedDeviceIndex, trackingSystemName.c_str(),
            renderModelName.c_str());
  } else {
    trackedDeviceToRenderModel_[trackedDeviceIndex] = renderModel;
    showTrackedDevice_[trackedDeviceIndex] = true;
  }
}

// Create/destroy D3D12 Render Models
void MainApplication::SetupRenderModels() {
  memset(trackedDeviceToRenderModel_, 0, sizeof(trackedDeviceToRenderModel_));

  if (!hmd_) return;

  for (uint32_t trackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1;
       trackedDevice < vr::k_unMaxTrackedDeviceCount; trackedDevice++) {
    if (!hmd_->IsTrackedDeviceConnected(trackedDevice)) continue;

    SetupRenderModelForTrackedDevice(trackedDevice);
  }
}

// Converts a SteamVR matrix to our local matrix class
Matrix4 MainApplication::ConvertSteamVRMatrixToMatrix4(
    const vr::HmdMatrix34_t& matPose) {
  Matrix4 matrixObj(matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
                    matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
                    matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
                    matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f);
  return matrixObj;
}