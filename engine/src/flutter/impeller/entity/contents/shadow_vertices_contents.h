// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_CONTENTS_SHADOW_VERTICES_CONTENTS_H_
#define FLUTTER_IMPELLER_ENTITY_CONTENTS_SHADOW_VERTICES_CONTENTS_H_

#include <memory>

#include "impeller/entity/contents/contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/shadow_path_geometry.h"
#include "impeller/geometry/color.h"

namespace impeller {

/// A vertices contents for (optional) per-color vertices + texture and any
/// blend mode.
class ShadowVerticesContents final : public Contents {
 public:
  static std::shared_ptr<ShadowVerticesContents> Make(
      const ShadowPathGeometry* geometry,
      Color shadow_color);

  void SetEffectTransform(Matrix transform);

  // |Contents|
  std::optional<Rect> GetCoverage(const Entity& entity) const override;

  // |Contents|
  bool Render(const ContentContext& renderer,
              const Entity& entity,
              RenderPass& pass) const override;

  ShadowVerticesContents(const ShadowPathGeometry* geometry,
                         Color shadow_color);

  ~ShadowVerticesContents() override;

 private:
  const ShadowPathGeometry* geometry_;
  Matrix inverse_matrix_;
  Color shadow_color_;

  ShadowVerticesContents(const ShadowVerticesContents&) = delete;

  ShadowVerticesContents& operator=(const ShadowVerticesContents&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_CONTENTS_SHADOW_VERTICES_CONTENTS_H_
