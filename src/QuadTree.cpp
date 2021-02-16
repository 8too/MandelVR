#include "QuadTree.h"

#include <complex>
#include <mutex>

#include "ExtendList.h"
#include "VertexArray.h"

// Constructor
QuadTree::QuadTree(QuadTree* parent, float x, float z, int level) {
  level_ = level;
  halfWidth_ = 4.0f / pow(2.0f, level);

  center_.x = x;
  center_.y = 0.0;
  center_.z = z;

  aabbMin_.x = x - halfWidth_;
  aabbMin_.y = -1.0;
  aabbMin_.z = z - halfWidth_;

  aabbMax_.x = x + halfWidth_;
  aabbMax_.y = 1.0;
  aabbMax_.z = z + halfWidth_;

  max_.y = -1.0;

  for (int i = 0; i < 4; i++) {
    children_[i] = nullptr;
  }

  vertexRef_.bufferId = -1;
  vertexRef_.vertexIndex = -1;

  status_ = QUAD_TREE_CREATED;
  parent_ = parent;
  numChildrenCommitted_ = 0;
}

QuadTree::~QuadTree() {
  for (int i = 0; i < 4; i++) {
    if (children_[i] != nullptr) {
      delete children_[i];
      children_[i] = nullptr;
    }
  }
}

// Set up quadtree with max level
void QuadTree::Setup(int maxLevel, thread_pool* threadPool,
                     VertexArray* vertexArray) {
  float widthX = 2.0f * halfWidth_ / kQuadTreeNodeResolution;
  float widthZ = 2.0f * halfWidth_ / kQuadTreeNodeResolution;
  float minY = 1.0;
  float maxY = -1.0;

  float heightMap[kHeightMapSize];
  for (int z = 0; z < kQuadTreeNodeResolution + 1; z++) {
    for (int x = 0; x < kQuadTreeNodeResolution + 1; x++) {
      float y =
          ComputeMandelHeight(aabbMin_.x + x * widthX, aabbMin_.z + z * widthZ);
      if (y < minY) {
        minY = y;
      }
      if (y > maxY) {
        maxY = y;
      }

      heightMap[z * (kQuadTreeNodeResolution + 1) + x] = y;
    }
  }

  bool isLeaf = (level_ >= maxLevel) || (maxY - minY <= 0.002f);
  if (!isLeaf) {
    float quarter_width = halfWidth_ / 2.0f;
    children_[0] = new QuadTree(this, center_.x - quarter_width,
                                center_.z - quarter_width, level_ + 1);
    children_[1] = new QuadTree(this, center_.x - quarter_width,
                                center_.z + quarter_width, level_ + 1);
    children_[2] = new QuadTree(this, center_.x + quarter_width,
                                center_.z - quarter_width, level_ + 1);
    children_[3] = new QuadTree(this, center_.x + quarter_width,
                                center_.z + quarter_width, level_ + 1);

    for (int i = 0; i < 4; i++) {
      QuadTree* child = children_[i];
      if (threadPool != nullptr) {
        std::function<void()> work_fun = [maxLevel, threadPool, vertexArray,
                                          child]() {
          child->Setup(maxLevel, threadPool, vertexArray);
        };
        threadPool->enqueue_task(work_fun);
      } else {
        child->Setup(maxLevel, threadPool, vertexArray);
      }
      numChildrenCommitted_++;
    }
  }

  std::vector<float> vertices;
  BuildTriangles(heightMap, & vertices);
  vertexRef_ = vertexArray->AddVertices(this, vertices);
}

void QuadTree::BuildTriangles(float* heightMap, std::vector<float>* vertices) {
  Vector3 cellVertices[kQuadTreeNodeResolution + 1]
                      [kQuadTreeNodeResolution + 1];
  float startX = aabbMin_.x;
  float startZ = aabbMin_.z;
  float widthX = 2.0f * halfWidth_ / kQuadTreeNodeResolution;
  float widthZ = 2.0f * halfWidth_ / kQuadTreeNodeResolution;

  for (int z = 0; z < kQuadTreeNodeResolution + 1; z++) {
    for (int x = 0; x < kQuadTreeNodeResolution + 1; x++) {
      cellVertices[z][x].x = startX + x * widthX;
      cellVertices[z][x].y = heightMap[z * (kQuadTreeNodeResolution + 1) +
                                       x];  // heightMap_[z][x];
      cellVertices[z][x].z = startZ + z * widthZ;
    }
  }

  for (int z = 0; z < kQuadTreeNodeResolution; z++) {
    for (int x = 0; x < kQuadTreeNodeResolution; x++) {
      if ((x + z) % 2 == 0) {
        vertices->push_back(cellVertices[z][x].x);
        vertices->push_back(cellVertices[z][x].y);
        vertices->push_back(cellVertices[z][x].z);

        vertices->push_back(cellVertices[z + 1][x + 1].x);
        vertices->push_back(cellVertices[z + 1][x + 1].y);
        vertices->push_back(cellVertices[z + 1][x + 1].z);

        vertices->push_back(cellVertices[z][x + 1].x);
        vertices->push_back(cellVertices[z][x + 1].y);
        vertices->push_back(cellVertices[z][x + 1].z);

        vertices->push_back(cellVertices[z][x].x);
        vertices->push_back(cellVertices[z][x].y);
        vertices->push_back(cellVertices[z][x].z);

        vertices->push_back(cellVertices[z + 1][x].x);
        vertices->push_back(cellVertices[z + 1][x].y);
        vertices->push_back(cellVertices[z + 1][x].z);

        vertices->push_back(cellVertices[z + 1][x + 1].x);
        vertices->push_back(cellVertices[z + 1][x + 1].y);
        vertices->push_back(cellVertices[z + 1][x + 1].z);
      } else {
        vertices->push_back(cellVertices[z][x].x);
        vertices->push_back(cellVertices[z][x].y);
        vertices->push_back(cellVertices[z][x].z);

        vertices->push_back(cellVertices[z + 1][x].x);
        vertices->push_back(cellVertices[z + 1][x].y);
        vertices->push_back(cellVertices[z + 1][x].z);

        vertices->push_back(cellVertices[z][x + 1].x);
        vertices->push_back(cellVertices[z][x + 1].y);
        vertices->push_back(cellVertices[z][x + 1].z);

        vertices->push_back(cellVertices[z + 1][x].x);
        vertices->push_back(cellVertices[z + 1][x].y);
        vertices->push_back(cellVertices[z + 1][x].z);

        vertices->push_back(cellVertices[z + 1][x + 1].x);
        vertices->push_back(cellVertices[z + 1][x + 1].y);
        vertices->push_back(cellVertices[z + 1][x + 1].z);

        vertices->push_back(cellVertices[z][x + 1].x);
        vertices->push_back(cellVertices[z][x + 1].y);
        vertices->push_back(cellVertices[z][x + 1].z);
      }
    }
  }
}

CellExtension QuadTree::ComputeExtension(const Vector3& cameraPos) {
  CellExtension extension;
  extension.aabbMin = aabbMin_;
  extension.aabbMax = aabbMax_;
  extension.cell = this;

  Vector3 distVec;
  if (cameraPos.x <= aabbMin_.x) {
    distVec.x = aabbMin_.x - cameraPos.x;
  } else if (cameraPos.x >= aabbMax_.x) {
    distVec.x = cameraPos.x - aabbMax_.x;
  } else {
    distVec.x = 0.0f;
  }

  if (cameraPos.y <= aabbMin_.y) {
    distVec.y = aabbMin_.y - cameraPos.y;
  } else if (cameraPos.y >= aabbMax_.y) {
    distVec.y = cameraPos.y - aabbMax_.y;
  } else {
    distVec.y = 0.0f;
  }

  if (cameraPos.z <= aabbMin_.z) {
    distVec.z = aabbMin_.z - cameraPos.z;
  } else if (cameraPos.z >= aabbMax_.z) {
    distVec.z = cameraPos.z - aabbMax_.z;
  } else {
    distVec.z = 0.0f;
  }
  extension.distToCamera = distVec.length();

  return extension;
}

void QuadTree::Extend(ExtendList* extendList, const Vector3& cameraPos) {
  if (children_[0] != nullptr) {
    dprintf("Cell already extended!");
    exit(0);
  }

  { 
    std::unique_lock lock(mutex_);

    float quarterWidth = halfWidth_ / 2.0f;
    children_[0] = new QuadTree(this, center_.x - quarterWidth,
                                center_.z - quarterWidth, level_ + 1);
    children_[1] = new QuadTree(this, center_.x - quarterWidth,
                                center_.z + quarterWidth, level_ + 1);
    children_[2] = new QuadTree(this, center_.x + quarterWidth,
                                center_.z - quarterWidth, level_ + 1);
    children_[3] = new QuadTree(this, center_.x + quarterWidth,
                                center_.z + quarterWidth, level_ + 1);
  }

  std::vector<CellExtension> extensions;
  for (int i = 0; i < 4; i++) {
    extensions.push_back(children_[i]->ComputeExtension(cameraPos));
  }
  extendList->AddCellExtensions(extensions);
}

const std::vector<float> QuadTree::Compute() {
  float widthX = 2.0f * halfWidth_ / kQuadTreeNodeResolution;
  float widthZ = 2.0f * halfWidth_ / kQuadTreeNodeResolution;
  float minY = 1.0;
  float maxY = -1.0;

  float heightMap[kHeightMapSize];
  for (int z = 0; z < kQuadTreeNodeResolution + 1; z++) {
    for (int x = 0; x < kQuadTreeNodeResolution + 1; x++) {
      float y =
          ComputeMandelHeight(aabbMin_.x + x * widthX, aabbMin_.z + z * widthZ);
      if (y < minY) {
        minY = y;
      }
      if (y > maxY) {
        maxY = y;
      }

      heightMap[z * (kQuadTreeNodeResolution + 1) + x] = y;
    }
  }

  std::vector<float> vertices;
  BuildTriangles(heightMap, &vertices);

  return vertices;
}

void QuadTree::NotifyCommitted(const VertexRef& vertexRef) {
  { 
    std::unique_lock lock(mutex_);

    vertexRef_ = vertexRef;
    status_ = QUAD_TREE_COMMITTED;
  }

  if (parent_ != nullptr) {
    parent_->NotifyChildCommitted();
  }
}

void QuadTree::NotifyChildCommitted() {
  std::unique_lock lock(mutex_);

  numChildrenCommitted_++;
}

void QuadTree::Render(const RenderingConfig& config, ExtendList* extendList,
                      VertexArray* vertices) {
  bool isCommitted = false;
  int numChildrenCommitted = -1;
  QuadTree* children[4];

  {
    std::unique_lock lock(mutex_); 

    isCommitted = (status_ == QUAD_TREE_COMMITTED);
    numChildrenCommitted = numChildrenCommitted_;
    for (int i = 0; i < 4; i++) {
      children[i] = children_[i];
    }
  }

  if (!isCommitted) {
    return;
  }

  if (level_ > config.GetMaxLevel()) {
    vertices->Render(vertexRef_);
    return;
  }

  const std::vector<Vector4> frustum = config.GetFrustum();

  if (config.IsCullingEnabled() && CanBeCulled(frustum)) {
    if (level_ == 0) {
      vertices->Render(vertexRef_);
    }
    return;
  }

  if (config.IsLodEnabled()) {
    const Vector3& cameraPos = config.GetCameraPos();
    const std::vector<float>& lodDistances = config.GetLodDistances();

    Vector3 distVec;
    if (cameraPos.x <= aabbMin_.x) {
      distVec.x = aabbMin_.x - cameraPos.x;
    } else if (cameraPos.x >= aabbMax_.x) {
      distVec.x = cameraPos.x - aabbMax_.x;
    } else {
      distVec.x = 0.0f;
    }

    if (cameraPos.y <= aabbMin_.y) {
      distVec.y = aabbMin_.y - cameraPos.y;
    } else if (cameraPos.y >= aabbMax_.y) {
      distVec.y = cameraPos.y - aabbMax_.y;
    } else {
      distVec.y = 0.0f;
    }

    if (cameraPos.z <= aabbMin_.z) {
      distVec.z = aabbMin_.z - cameraPos.z;
    } else if (cameraPos.z >= aabbMax_.z) {
      distVec.z = cameraPos.z - aabbMax_.z;
    } else {
      distVec.z = 0.0f;
    }

    float dist = distVec.length();
    if (dist > lodDistances[level_]) {
      vertices->Render(vertexRef_);
      return;
    }
  }

  if (children[0] == nullptr || numChildrenCommitted < 4) {
    vertices->Render(vertexRef_);
    if (children[0] == nullptr) {
      Extend(extendList, config.GetCameraPos());
    }
  } else {
    for (int i = 0; i < 4; i++) {
      children[i]->Render(config, extendList, vertices);
    }
  }
}

bool QuadTree::CanBeCulled(const std::vector<Vector4>& frustum) {
  bool cull = false;

  for (int planeID = 0; planeID < 6; ++planeID) {
    const Vector4* plane = &frustum[planeID];
    Vector3 plane_normal(plane->x, plane->y, plane->z);
    Vector3 closestVertex;

    // x-axis
    if (plane->x < 0.0f)  // Which AABB vertex is furthest down (plane normals
                          // direction) the x axis
      closestVertex.x = aabbMin_.x;  // min x
    else
      closestVertex.x = aabbMax_.x;  // max x

    // y-axis
    if (plane->y < 0.0f)  // Which AABB vertex is furthest down (plane normals
                          // direction) the y axis
      closestVertex.y = aabbMin_.y;  // min y
    else
      closestVertex.y = aabbMax_.y;  // max y

    // z-axis
    if (plane->z < 0.0f)  // Which AABB vertex is furthest down (plane normals
                          // direction) the x axis
      closestVertex.z = aabbMin_.z;  // min z
    else
      closestVertex.z = aabbMax_.z;  // max z

    // Now we get the signed distance from the AABB vertex that's furthest down
    // the frustum planes normal, and if the signed distance is negative, then
    // the entire bounding box is behind the frustum plane, which means that it
    // should be culled
    if (plane_normal.dot(closestVertex) + plane->w < 0.0f) {
      cull = true;
      // Skip remaining planes to check and move on to next tree
      break;
    }
  }

  return cull;
}

float QuadTree::ComputeMandelHeight(float posX, float posY) {
  double x, xx, y, cx, cy;
  int iteration;
  int itermax = 100;    // 100;		/* how many iterations to do	*/
  double magnify = 2.0; /* double magnification		*/

  cx = posX / magnify - 0.7;
  cy = posY / magnify;
  x = 0.0;
  y = 0.0;
  for (iteration = 1; iteration < itermax; iteration++) {
    xx = x * x - y * y + cx;
    y = 2.0 * x * y + cy;
    x = xx;
    if (x * x + y * y > 16.0) return (float)iteration / itermax;
  }
  return 1.0f;
}

float QuadTree::ComputeMandelDistance(float posX, float posY) {
  int iteration;
  int itermax = 100;    /* how many iterations to do	*/
  double magnify = 2.0; /* double magnification		*/

  std::complex<double> c(posX / magnify - 0.7, posY / magnify);
  std::complex<double> z = c;
  std::complex<double> dz(0.0, 0.0);
  for (iteration = 1; iteration < itermax; iteration++) {
    std::complex<double> z2 = z * z + c;
    dz = 2.0 * z * dz + 1.0;
    z = z2;

    if (std::norm(z) > 16.0) {
      double dist = log(norm(z)) * abs(z) / abs(dz);
      return 1.0 / ((1.0 + dist) * (1.0 + dist));
    }
  }
  return 1.0;
}