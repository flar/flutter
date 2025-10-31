// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/dl_sampling_options.h"
#include "display_list/dl_tile_mode.h"
#include "display_list/effects/dl_color_source.h"
#include "display_list/effects/dl_mask_filter.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/display_list/geometry/dl_path_builder.h"
#include "flutter/impeller/entity/geometry/shadow_path_geometry.h"
#include "flutter/testing/testing.h"
#include "impeller/display_list/dl_image_impeller.h"
#include "impeller/playground/widgets.h"
#include "impeller/tessellator/path_tessellator.h"

namespace impeller {
namespace testing {

using namespace flutter;

namespace {
  void DrawShadowMesh(DisplayListBuilder& builder,
                      const DlPath& path,
                      Scalar elevation,
                      Scalar dpr,
                      bool use_skia) {
    std::shared_ptr<ShadowVertices> shadow_vertices;
    DlPaint paint;
    paint.setDrawStyle(DlDrawStyle::kStroke);
    if (use_skia) {
#ifndef NDEBUG
      shadow_vertices = ShadowPathGeometry::MakeAmbientShadowVerticesSkia(
          path, elevation, {});
      paint.setColor(DlColor::kGreen());
#else
      return;
#endif
    } else {
      Tessellator tessellator;
      shadow_vertices = ShadowPathGeometry::MakeAmbientShadowVertices(
          tessellator, path, elevation, {});
      ASSERT_TRUE(shadow_vertices);
      paint.setColor(DlColor::kRed());
    }

    builder.Save();
    builder.Translate(0, elevation * dpr * 0.5f);
    ASSERT_TRUE(shadow_vertices);
    auto indices = shadow_vertices->GetIndices();
    auto vertices = shadow_vertices->GetVertices();
    DlPathBuilder mesh_builder;
    for (size_t i = 0; i < shadow_vertices->GetIndexCount(); i += 3) {
      mesh_builder.MoveTo(vertices[indices[i + 0]]);
      mesh_builder.LineTo(vertices[indices[i + 1]]);
      mesh_builder.LineTo(vertices[indices[i + 2]]);
      mesh_builder.Close();
    }
    DlPath mesh_path = mesh_builder.TakePath();
    builder.DrawPath(mesh_path, paint);

    paint.setColor(paint.getColor().withAlphaF(0.5f));
    builder.DrawPath(path, paint);
    builder.Restore();
  }
}  // namespace

TEST_P(AiksTest, CanDrawClockwiseTriangleShadow) {
  DisplayListBuilder builder;
  builder.Clear(DlColor::kWhite());
  builder.Scale(GetContentScale().x, GetContentScale().y);
  Scalar dpr = std::max(GetContentScale().x, GetContentScale().y);
  Scalar elevation = 30.0f;

  DlPathBuilder triangle_builder;
  triangle_builder.MoveTo(DlPoint(200, 100));
  triangle_builder.LineTo(DlPoint(300, 200));
  triangle_builder.LineTo(DlPoint(100, 200));
  triangle_builder.Close();
  DlPath triangle_path = triangle_builder.TakePath();

  builder.DrawShadow(triangle_path, DlColor::kBlue(), elevation, true, dpr);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, false);
  builder.Translate(0, 300);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, true);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, false);
  builder.Translate(0, -300);

  builder.Translate(300, 0);
  builder.DrawShadow(triangle_path, DlColor::kBlue(), elevation, true, dpr);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, true);
  builder.Translate(0, 300);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, false);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, true);
  builder.Translate(0, -300);

  auto dl = builder.Build();
  ASSERT_TRUE(OpenPlaygroundHere(dl));
}

TEST_P(AiksTest, CanDrawCounterClockwiseTriangleShadow) {
  DisplayListBuilder builder;
  builder.Clear(DlColor::kWhite());
  builder.Scale(GetContentScale().x, GetContentScale().y);
  Scalar dpr = std::max(GetContentScale().x, GetContentScale().y);
  Scalar elevation = 30.0f;

  DlPathBuilder triangle_builder;
  triangle_builder.MoveTo(DlPoint(200, 100));
  triangle_builder.LineTo(DlPoint(100, 200));
  triangle_builder.LineTo(DlPoint(300, 200));
  triangle_builder.Close();
  DlPath triangle_path = triangle_builder.TakePath();

  builder.DrawShadow(triangle_path, DlColor::kBlue(), elevation, true, dpr);
  DrawShadowMesh(builder, triangle_path, 30.0f, dpr, false);
  builder.Translate(0, 300);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, true);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, false);
  builder.Translate(0, -300);

  builder.Translate(300, 0);
  builder.DrawShadow(triangle_path, DlColor::kBlue(), elevation, true, dpr);
  DrawShadowMesh(builder, triangle_path, 30.0f, dpr, true);
  builder.Translate(0, 300);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, false);
  DrawShadowMesh(builder, triangle_path, elevation, dpr, true);
  builder.Translate(0, -300);

  auto dl = builder.Build();
  ASSERT_TRUE(OpenPlaygroundHere(dl));
}

}  // namespace testing
}  // namespace impeller
