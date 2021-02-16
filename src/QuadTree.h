#pragma once

#include <mutex>
#include <vector>
#include <windows.h>
#include <Matrices.h>

#include "threads/threadpool.h"
#include "RenderingConfig.h"

const int kFloatsPerPoint = 3;
const int kQuadTreeNodeResolution = 32;
const int kHeightMapSize =
    (kQuadTreeNodeResolution + 1) * (kQuadTreeNodeResolution + 1);
const int kMaxNumLevels = 64;
const float kMinHeight = 0.01;

class VertexArray;
class ExtendList;
struct CellExtension;

struct VertexRef {
  int bufferId;
  int vertexIndex;
  int meshSize;
};

class QuadTree {
 public:
  QuadTree(QuadTree* parent, float x, float z, int level);
  ~QuadTree();

  void Setup(int maxLevel, thread_pool* thread_pool, VertexArray* vertices);
  void Render(const RenderingConfig& config, ExtendList* extend_list,
              VertexArray* vertices);

  const std::vector<float> Compute();
  void NotifyCommitted(const VertexRef& vertex_ref);
  void NotifyChildCommitted();

 private:
  void BuildTriangles(float* heightMap, std::vector<float>* vertices);

  bool CanBeCulled(const std::vector<Vector4>& frustum);
  float ComputeMandelHeight(float pos_x, float pos_y);
  float ComputeMandelDistance(float pos_x, float pos_y);

  CellExtension ComputeExtension(const Vector3& camera_pos);
  void Extend(ExtendList* extend_list, const Vector3& camera_pos);

  int level_;
  float halfWidth_;

  Vector3 center_;
  Vector3 aabbMin_;
  Vector3 aabbMax_;
  Vector3 max_;

  std::mutex mutex_;
  enum { QUAD_TREE_CREATED, QUAD_TREE_COMPUTED, QUAD_TREE_COMMITTED } status_;
  int numChildrenCommitted_;
  QuadTree* parent_;
  QuadTree* children_[4];
  VertexRef vertexRef_;
};
