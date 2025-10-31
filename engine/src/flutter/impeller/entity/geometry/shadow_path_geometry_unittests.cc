// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/entity/geometry/shadow_path_geometry.h"

#include "flutter/display_list/geometry/dl_path.h"
#include "flutter/display_list/geometry/dl_path_builder.h"
#include "gtest/gtest.h"

#include "flutter/third_party/skia/src/core/SkVerticesPriv.h"  // nogncheck
#include "flutter/third_party/skia/src/utils/SkShadowTessellator.h"  // nogncheck

namespace impeller {
namespace testing {

using namespace flutter;

namespace {

static void ShowVertices(std::string label,
                         std::shared_ptr<ShadowVertices> shadow_vertices) {
  auto vertices = shadow_vertices->GetVertices();
  auto alphas = shadow_vertices->GetGaussians();
  auto indices = shadow_vertices->GetIndices();
  FML_LOG(ERROR) << label << "[" << indices.size() / 3 << "] = {";
  for (size_t i = 0u; i < indices.size(); i += 3) {
    // clang-format off
    FML_LOG(ERROR)
        << "  (" << vertices[indices[i + 0]] << ", " << alphas[indices[i + 0]] << "), "
        << "  (" << vertices[indices[i + 1]] << ", " << alphas[indices[i + 1]] << "), "
        << "  (" << vertices[indices[i + 2]] << ", " << alphas[indices[i + 2]] << ")";
    // clang-format on
  }
  FML_LOG(ERROR) << "}  // " << label;
}

}  // namespace

TEST(ShadowPathGeometryTest, ClockwiseRectTest) {
  DlPathBuilder path_builder;
  path_builder.MoveTo(DlPoint(0, 0));
  path_builder.LineTo(DlPoint(100, 0));
  path_builder.LineTo(DlPoint(100, 80));
  path_builder.LineTo(DlPoint(0, 80));
  path_builder.Close();
  const DlPath path = path_builder.TakePath();
  const Matrix matrix;
  const Scalar height = 2.0f;

  Tessellator tessellator;
  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowPathGeometry::MakeAmbientShadowVertices(tessellator, path, height,
                                                    matrix);

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertexCount(), 22u);
  EXPECT_EQ(shadow_vertices->GetIndexCount(), 72u);
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 22u);
  EXPECT_EQ(shadow_vertices->GetGaussians().size(), 22u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 72u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);

  ShowVertices("Impeller Vertices", shadow_vertices);

#ifndef NDEBUG
  auto sk_shadow_vertices = ShadowPathGeometry::MakeAmbientShadowVerticesSkia(
      path, height, matrix);
  ShowVertices("Skia Vertices", sk_shadow_vertices);
#endif
}

TEST(ShadowPathGeometryTest, CounterClockwiseRectTest) {
  DlPathBuilder path_builder;
  path_builder.MoveTo(DlPoint(0, 0));
  path_builder.LineTo(DlPoint(0, 80));
  path_builder.LineTo(DlPoint(100, 80));
  path_builder.LineTo(DlPoint(100, 0));
  path_builder.Close();
  DlPath path = path_builder.TakePath();
  Matrix matrix;
  const Scalar height = 2.0f;

  Tessellator tessellator;
  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowPathGeometry::MakeAmbientShadowVertices(tessellator, path, height,
                                                    matrix);

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertexCount(), 22u);
  EXPECT_EQ(shadow_vertices->GetIndexCount(), 72u);
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 22u);
  EXPECT_EQ(shadow_vertices->GetGaussians().size(), 22u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 72u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);

  ShowVertices("Impeller Vertices", shadow_vertices);
}

TEST(ShadowPathGeometryTest, ScaledRectTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeRect(DlRect::MakeLTRB(0, 0, 100, 80));
  Matrix matrix = Matrix::MakeScale({2, 3, 1});
  const Scalar height = 2.0f;

  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowPathGeometry::MakeAmbientShadowVertices(tessellator, path, height,
                                                    matrix);

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertexCount(), 22u);
  EXPECT_EQ(shadow_vertices->GetIndexCount(), 72u);
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 22u);
  EXPECT_EQ(shadow_vertices->GetGaussians().size(), 22u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 72u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);

  ShowVertices("Impeller Vertices", shadow_vertices);
}

TEST(ShadowPathGeometryTest, EllipseTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeOval(DlRect::MakeLTRB(0, 0, 100, 80));
  Matrix matrix;
  const Scalar height = 2.0f;

  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowPathGeometry::MakeAmbientShadowVertices(tessellator, path, height,
                                                    matrix);

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertexCount(), 162u);
  EXPECT_EQ(shadow_vertices->GetIndexCount(), 600u);
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 162u);
  EXPECT_EQ(shadow_vertices->GetGaussians().size(), 162u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 600u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);
}

TEST(ShadowPathGeometryTest, RoundRectTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeRoundRectXY(DlRect::MakeLTRB(0, 0, 100, 80), 5, 4);
  Matrix matrix;
  const Scalar height = 2.0f;

  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowPathGeometry::MakeAmbientShadowVertices(tessellator, path, height,
                                                    matrix);

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertexCount(), 66u);
  EXPECT_EQ(shadow_vertices->GetIndexCount(), 240u);
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 66u);
  EXPECT_EQ(shadow_vertices->GetGaussians().size(), 66u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 240u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);
}

}  // namespace testing
}  // namespace impeller
