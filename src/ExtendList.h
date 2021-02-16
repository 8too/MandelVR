#pragma once

#include <queue>
#include <vector>
#include <Matrices.h>

#include "QuadTree.h"
#include "util.h"

struct CellExtension {
  Vector3 aabbMin;
  Vector3 aabbMax;
  class QuadTree* cell;
  float distToCamera;
};

struct CellExtensionLess {
  constexpr bool operator()(const CellExtension& lhs,
                            const CellExtension& rhs) const {
    return lhs.distToCamera < rhs.distToCamera;
  }
};

class ExtendList {
 public:
  ExtendList() { }

  ~ExtendList() { }

  void AddCellExtensions(const std::vector<CellExtension>& extensions) {
    std::unique_lock lock(mutex_);

    for (const CellExtension& extension : extensions) {
      extensionsQueue_.push(extension);
    }
  }

  QuadTree* PopCell() {
    QuadTree* cell = nullptr;

    std::unique_lock lock(mutex_);

    if (!extensionsQueue_.empty()) {
      cell = extensionsQueue_.top().cell;
      extensionsQueue_.pop();
    }

    return cell;
  }

  int Size() { 
    std::unique_lock lock(mutex_);
    return extensionsQueue_.size();
  }

 private:
  std::mutex mutex_;
  std::priority_queue<CellExtension, std::vector<CellExtension>,
                      CellExtensionLess>
      extensionsQueue_;
};
