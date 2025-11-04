// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "shadow_vertices_contents.h"

#include <format>

#include "fml/logging.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/contents/filters/blend_filter_contents.h"
#include "impeller/entity/contents/pipelines.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/entity/geometry/vertices_geometry.h"
#include "impeller/geometry/color.h"
#include "impeller/renderer/render_pass.h"

namespace impeller {

//------------------------------------------------------
// ShadowVerticesContents

ShadowVerticesContents::ShadowVerticesContents() {}

ShadowVerticesContents::~ShadowVerticesContents() {}

void ShadowVerticesContents::SetGeometry(
    std::shared_ptr<ShadowVertices> geometry) {
  geometry_ = std::move(geometry);
}

std::optional<Rect> ShadowVerticesContents::GetCoverage(
    const Entity& entity) const {
  return geometry_->GetBounds();
}

void ShadowVerticesContents::SetEffectTransform(Matrix transform) {
  inverse_matrix_ = transform.Invert();
}

bool ShadowVerticesContents::Render(const ContentContext& renderer,
                                    const Entity& entity,
                                    RenderPass& pass) const {
#if NOT_YET_READY
  BlendMode blend_mode = blend_mode_;

  auto dst_sampler_descriptor = descriptor_;

  GeometryResult geometry_result = geometry_->GetPositionUVColorBuffer(
      lazy_texture_coverage_.has_value() ? lazy_texture_coverage_.value()
                                         : Rect::MakeSize(texture->GetSize()),
      inverse_matrix_, renderer, entity, pass);
  if (geometry_result.vertex_buffer.vertex_count == 0) {
    return true;
  }
  FML_DCHECK(geometry_result.mode == GeometryResult::Mode::kNormal);

  if (blend_mode <= Entity::kLastPipelineBlendMode) {
    using VS = PorterDuffBlendPipeline::VertexShader;
    using FS = PorterDuffBlendPipeline::FragmentShader;

#ifdef IMPELLER_DEBUG
    pass.SetCommandLabel(std::format("DrawVertices Porterduff Blend ({})",
                                     BlendModeToString(blend_mode)));
#endif  // IMPELLER_DEBUG
    pass.SetVertexBuffer(std::move(geometry_result.vertex_buffer));

    auto options = OptionsFromPassAndEntity(pass, entity);
    options.primitive_type = geometry_result.type;
    auto inverted_blend_mode =
        InvertPorterDuffBlend(blend_mode).value_or(BlendMode::kSrc);
    pass.SetPipeline(
        renderer.GetPorterDuffPipeline(inverted_blend_mode, options));

    FS::BindTextureSamplerDst(pass, texture, dst_sampler);

    VS::FrameInfo frame_info;
    FS::FragInfo frag_info;

    frame_info.texture_sampler_y_coord_scale = texture->GetYCoordScale();
    frame_info.mvp = geometry_result.transform;

    frag_info.input_alpha_output_alpha_tmx_tmy =
        Vector4(1, alpha_, static_cast<int>(tile_mode_x_),
                static_cast<int>(tile_mode_y_));
    frag_info.use_strict_source_rect = 0.0;

    auto& host_buffer = renderer.GetTransientsDataBuffer();
    FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
    VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

    return pass.Draw().ok();
  }

  using VS = VerticesUber1Shader::VertexShader;
  using FS = VerticesUber1Shader::FragmentShader;

#ifdef IMPELLER_DEBUG
  pass.SetCommandLabel(std::format("DrawVertices Advanced Blend ({})",
                                   BlendModeToString(blend_mode)));
#endif  // IMPELLER_DEBUG
  pass.SetVertexBuffer(std::move(geometry_result.vertex_buffer));

  auto options = OptionsFromPassAndEntity(pass, entity);
  options.primitive_type = geometry_result.type;
  pass.SetPipeline(renderer.GetDrawVerticesUberPipeline(blend_mode, options));

  FS::BindTextureSampler(pass, texture, dst_sampler);

  VS::FrameInfo frame_info;
  FS::FragInfo frag_info;

  frame_info.texture_sampler_y_coord_scale = texture->GetYCoordScale();
  frame_info.mvp = geometry_result.transform;
  frag_info.alpha = alpha_;
  frag_info.blend_mode = static_cast<int>(blend_mode);

  // These values are ignored if the platform supports native decal mode.
  frag_info.tmx = static_cast<int>(tile_mode_x_);
  frag_info.tmy = static_cast<int>(tile_mode_y_);

  auto& host_buffer = renderer.GetTransientsDataBuffer();
  FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));
  VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));

  return pass.Draw().ok();
#endif
  return false;
}

}  // namespace impeller
