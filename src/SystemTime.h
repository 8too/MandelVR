#pragma once

#include <windows.h>

#include <SDL.h>
#include <SDL_syswm.h>

#include "util.h"

class SystemTime {
 public:
  // Query the performance counter frequency
  static void Initialize(void) {
    LARGE_INTEGER frequency;
    if (!QueryPerformanceFrequency(&frequency)) {
      dprintf("Unable to query performance counter frequency.\n");
    }
    cpuTickDelta_ = 1.0 / static_cast<double>(frequency.QuadPart);
  }

  // Query the current value of the performance counter
  static int64_t GetCurrentTick(void) {
    LARGE_INTEGER currentTick;
    if (!QueryPerformanceCounter(&currentTick)) {
      dprintf("Unable to query performance counter value");
    }
    return static_cast<int64_t>(currentTick.QuadPart);
  }

  static inline double TicksToSeconds(int64_t TickCount) {
    return TickCount * cpuTickDelta_;
  }

  static inline double TicksToMillisecs(int64_t TickCount) {
    return TickCount * cpuTickDelta_ * 1000.0;
  }

  static inline double TimeBetweenTicks(int64_t tick1, int64_t tick2) {
    return TicksToSeconds(tick2 - tick1);
  }

 private:
  // The amount of time that elapses between ticks of the performance counter
  static double cpuTickDelta_;
};

class CpuTimer {
 public:
  CpuTimer() {
    startTick_ = 0ll;
    elapsedTicks_ = 0ll;
  }

  void Start() {
    if (startTick_ == 0ll) startTick_ = SystemTime::GetCurrentTick();
  }

  void Stop() {
    if (startTick_ != 0ll) {
      elapsedTicks_ += SystemTime::GetCurrentTick() - startTick_;
      startTick_ = 0ll;
    }
  }

  void Reset() {
    elapsedTicks_ = 0ll;
    startTick_ = 0ll;
  }

  double GetTime() const { return SystemTime::TicksToSeconds(elapsedTicks_); }

 private:
  int64_t startTick_;
  int64_t elapsedTicks_;
};
