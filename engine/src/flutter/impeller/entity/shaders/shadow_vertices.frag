// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <impeller/types.glsl>

// Compute a guassian value from an input coefficient taken from the
// interpolated alpha values of the vertices in the mesh.
// From Skia: src/opts/SkRasterPipeline_opts.h
float gauss(float a) {
  // x = 1 - x;
  // exp(-x * x * 4) - 0.018f;
  // ... now approximate with quartic
  //
  const float c4 = -2.26661229133605957031f;
  const float c3 = 2.89795351028442382812f;
  const float c2 = 0.21345567703247070312f;
  const float c1 = 0.15489584207534790039f;
  const float c0 = 0.00030726194381713867f;
  a = fma(a, fma(a, fma(a, fma(a, c4, c3), c2), c1), c0);
  return a;
}

uniform FragInfo {
  // shadow_color is the color supplied to DrawShadow. It will be modulated
  // by the gaussian opacity of the shadow, computed from the coefficient.
  vec4 shadow_color;
}
frag_info;

// v_color will contain the interpolated gaussian coefficient from the mesh
// as its alpha value. It determines where in the gaussian curve of the
// umbra and penumbra we are with 0.0 representing the outermost part of
// the penumbra and 1.0 representing the innermost umbra.
in float v_gaussian;

out f16vec4 frag_color;

// A shader that implements the required gaussian interpolation and then
// blending required for DrawShadow in a single step.
void main() {
  frag_color = frag_info.shadow_color * gauss(v_gaussian);
}
