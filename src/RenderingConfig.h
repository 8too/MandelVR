#pragma once

#include <vector>

#include <Matrices.h>

class RenderingConfig {
 public:
  static RenderingConfig Create(int maxLevel) { return RenderingConfig(maxLevel, false, false);
  }

  void AddFrustumCulling(const std::vector<Vector4>& planes,
                         const Vector3 camera_pos) {
    enableCulling_ = true;

    frustumPlanes_ = planes;
    cameraPos_ = camera_pos;
  }

  void EnableLod(const std::vector<float>& lod_distances) {
    enableLod_ = true;
    lodDistances_ = lod_distances;
  }

  bool IsCullingEnabled() const { return enableCulling_; }
  const std::vector<Vector4>& GetFrustum() const { return frustumPlanes_; }
  const Vector3& GetCameraPos() const { return cameraPos_; }

  bool IsLodEnabled() const { return enableLod_; }
  const std::vector<float>& GetLodDistances() const { return lodDistances_; }

  int GetMaxLevel() const { return maxLevel_; }

 private:
  RenderingConfig(int maxLevel, bool enable_culling, bool enable_lod)
      : maxLevel_(maxLevel), enableCulling_(enable_culling), enableLod_(enable_lod) {}

  bool enableCulling_;
  std::vector<Vector4> frustumPlanes_;
  Vector3 cameraPos_;
  int maxLevel_;

  bool enableLod_;
  std::vector<float> lodDistances_;
};
