// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/impeller/tessellator/shadow_tessellator.h"

#include "flutter/display_list/geometry/dl_path.h"
#include "flutter/display_list/geometry/dl_path_builder.h"

namespace impeller {
namespace testing {

using namespace flutter;

TEST(ShadowTessellatorTest, RectTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeRect(DlRect::MakeLTRB(0, 0, 100, 80));

  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowTessellator::MakeAmbientShadowVertices(tessellator, path, 2.0f, {});

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 14u);
  EXPECT_EQ(shadow_vertices->GetColors().size(), 14u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 33u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);
}

TEST(ShadowTessellatorTest, EllipseTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeOval(DlRect::MakeLTRB(0, 0, 100, 80));

  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowTessellator::MakeAmbientShadowVertices(tessellator, path, 2.0f, {});

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 198u);
  EXPECT_EQ(shadow_vertices->GetColors().size(), 198u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 585u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);
}

TEST(ShadowTessellatorTest, RoundRectTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeRoundRectXY(DlRect::MakeLTRB(0, 0, 100, 80), 5, 4);

  std::shared_ptr<ShadowVertices> shadow_vertices =
      ShadowTessellator::MakeAmbientShadowVertices(tessellator, path, 2.0f, {});

  ASSERT_NE(shadow_vertices, nullptr);
  EXPECT_FALSE(shadow_vertices->IsEmpty());
  EXPECT_EQ(shadow_vertices->GetVertices().size(), 78u);
  EXPECT_EQ(shadow_vertices->GetColors().size(), 78u);
  EXPECT_EQ(shadow_vertices->GetIndices().size(), 225u);
  EXPECT_EQ((shadow_vertices->GetIndices().size() % 3u), 0u);
}

}  // namespace testing
}  // namespace impeller
