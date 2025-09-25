// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TESSELLATOR_SHADOW_TESSELLATOR_H_
#define FLUTTER_IMPELLER_TESSELLATOR_SHADOW_TESSELLATOR_H_

#include "flutter/impeller/geometry/path_source.h"

namespace impeller {

class ShadowTessellator {
 public:
  std::shared_ptr<ShadowTessellator> MakeAmbientShadowVertices(
      const PathSource& source,
      Scalar occluder_height,
      const Matrix& matrix);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TESSELLATOR_SHADOW_TESSELLATOR_H_
