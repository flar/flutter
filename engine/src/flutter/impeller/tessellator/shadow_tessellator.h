// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TESSELLATOR_SHADOW_TESSELLATOR_H_
#define FLUTTER_IMPELLER_TESSELLATOR_SHADOW_TESSELLATOR_H_

#include "flutter/impeller/geometry/path_source.h"
#include "flutter/impeller/tessellator/tessellator.h"

namespace impeller {

class ShadowVertices {
 public:
  constexpr ShadowVertices(std::vector<Point> vertices,
                           std::vector<int> indices,
                           std::vector<Color> colors)
      : vertices_(std::move(vertices)),
        indices_(std::move(indices)),
        colors_(std::move(colors)) {}

  const std::vector<Point>& GetVertices() const { return vertices_; }
  const std::vector<int>& GetIndices() const { return indices_; }
  const std::vector<Color>& GetColors() const { return colors_; }

  bool IsEmpty() const { return vertices_.empty(); }

 private:
  const std::vector<Point> vertices_;
  const std::vector<int> indices_;
  const std::vector<Color> colors_;
};

class ShadowTessellator {
 public:
  static std::shared_ptr<ShadowVertices> MakeAmbientShadowVertices(
      Tessellator& tessellator,
      const PathSource& source,
      Scalar occluder_height,
      const Matrix& matrix);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TESSELLATOR_SHADOW_TESSELLATOR_H_
