// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <impeller/types.glsl>

uniform FrameInfo {
  mat4 mvp;
}
frame_info;

in vec2 position;

out f16vec4 v_color;

void main() {
  // Shadow vertices geometry is already in device space.
  gl_Position = vec4(position, 0.0, 1.0);
}
