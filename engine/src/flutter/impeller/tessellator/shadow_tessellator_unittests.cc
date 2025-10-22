// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/impeller/tessellator/shadow_tessellator.h"

#include "flutter/display_list/geometry/dl_path.h"
#include "flutter/display_list/geometry/dl_path_builder.h"

namespace impeller {
namespace testing {

using flutter::DlPath;
using flutter::DlRect;

TEST(ShadowTessellatorTest, EllipseTest) {
  Tessellator tessellator;
  DlPath path = DlPath::MakeOval(DlRect::MakeLTRB(0, 0, 100, 100));

  std::shared_ptr<ShadowVertices> vertices =
      ShadowTessellator::MakeAmbientShadowVertices(tessellator, path, 2.0f, {});

  ASSERT_NE(vertices, nullptr);
  EXPECT_FALSE(vertices->IsEmpty());
  ASSERT_NE(vertices->GetVertices().size(), 0u);
  ASSERT_NE(vertices->GetColors().size(), 0u);
  ASSERT_NE(vertices->GetIndices().size(), 0u);
}

}  // namespace testing
}  // namespace impeller
