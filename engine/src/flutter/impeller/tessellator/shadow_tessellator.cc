// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/tessellator/shadow_tessellator.h"

#include "flutter/impeller/tessellator/path_tessellator.h"

namespace {

using Point = impeller::Point;
using Scalar = impeller::Scalar;
using Vector2 = impeller::Vector2;

class PolygonInfo : impeller::PathTessellator::VertexWriter {
 public:
  PolygonInfo(const impeller::PathSource& path, const impeller::Matrix& matrix);

  bool IsValid() const { return is_valid_; }
  const std::vector<Point> GetVertices() const { return points_; }

 private:
  const impeller::Matrix& matrix_;

  std::vector<Point> points_;
  bool is_valid_;  // aka single convex contour

  Point centroid_;
  Scalar area_;
  Scalar direction_;
  bool ended_;

  // |VertexWriter|
  void Write(Point point);

  // |VertexWriter|
  void EndContour();

  // Parameters that determine the sub-pixel grid we will use to simplify
  // the contours to avoid degenerate differences in the vertices.
  static constexpr Scalar sub_pixel_count_ = 16.0f;
  static constexpr Scalar sub_pixel_scale_ = (1.0f / sub_pixel_count_);

  // Transforms point and rounds to the sub-pixel grid
  Point AdjustPoint(Point point);

  // Updates the area and centroid point based on the partial areas of the
  // new point, the previous point and the first point.
  //
  // The method also tests for multiple conditions of convexity. If the last
  // 3 points are not turning the same direction as previous points, then the
  // path is locally not convex and therefore "invalid". If the quad area of
  // the last 2 points wrt the first point (computed for area and centroid)
  // is not the same sign as the contour's direction then the path is now
  // progressing the "wrong way" around the initial point which indicates
  // multiple turns and self-intersection and is thus "invalid".
  //
  // This method will DCHECK that the points vector has at least 2 points.
  bool ComputeAndValidateCentroid(const Point& new_point);
};

PolygonInfo::PolygonInfo(const impeller::PathSource& source,
                         const impeller::Matrix& matrix)
    : matrix_(matrix),
      is_valid_(true),
      centroid_({}),
      area_(0.0f),
      direction_(0.0f),
      ended_(false) {
  Scalar scale = matrix.GetMaxBasisLengthXY();
  auto [point_count, contour_count] =
      impeller::PathTessellator::CountFillStorage(source, scale);
  points_.reserve(point_count);
  impeller::PathTessellator::PathToFilledVertices(source, *this, scale);
}

void PolygonInfo::Write(Point point) {
  if (ended_) {
    is_valid_ = false;
  }
  if (!is_valid_) {
    return;
  }

  point = AdjustPoint(point);

  if (!points_.empty()) {
    if (point == points_.back()) {
      // Avoid duplicate points, adjusted points are rounded so == is OK
      return;
    }
    if (!ComputeAndValidateCentroid(point)) {
      FML_DCHECK(!is_valid_);
      return;
    }
  }

  points_.emplace_back(point);
}

void PolygonInfo::EndContour() {
  if (ended_) {
    is_valid_ = false;
    return;
  }

  // PathTessellator always ensures the path is closed back to the origin.
  FML_DCHECK(points_.front() == points_.back());
  points_.pop_back();
  ended_ = true;
}

Point PolygonInfo::AdjustPoint(Point point) {
  // Transform to device space and round to nearest sub-pixel.
  return ((matrix_ * point) * sub_pixel_count_).Round() * sub_pixel_scale_;
}

bool PolygonInfo::ComputeAndValidateCentroid(const Point& new_point) {
  if (!is_valid_) {
    return false;
  }
  if (points_.size() < 2u) {
    return true;
  }

  const Point& prev = points_.back();

  if (direction_ != 0) {
    // Note that if there are only 2 points in the path, the direction_
    // will not yet have been determined and this check computes the same
    // values as the global check below anyway.
    FML_DCHECK(points_.size() > 2u);

    // The area convexity check below only checks if the contour goes around
    // once, but that contour could still contain locally concave points that
    // happen to continue their path around the initial point in the same
    // direction. (Consider a circle with a dent on the far side of it, the
    // points all progress in one direction from the first point even though
    // the 3 vertices surrounding the dent are themselves concave).

    const Point& prev_prev = points_.end()[-2];
    Vector2 v0 = prev - prev_prev;
    Vector2 v1 = new_point - prev_prev;
    if (v0.Cross(v1) * direction_ < 0) {
      is_valid_ = false;
      return false;
    }
  }

  const Point& first = points_.front();
  Vector2 v0 = prev - first;
  Vector2 v1 = new_point - first;
  Scalar quad_area = v0.Cross(v1);

  // convexity check for whole path which can detect if we turn more than
  // 360 degrees and start going the other way wrt the start point, but
  // does not detect if any pair of points are concave (checked above).
  if (direction_ == 0) {
    direction_ = quad_area;
  } else if (quad_area * direction_ < 0) {
    is_valid_ = false;
    return false;
  }

  centroid_ += (v0 + v1) * quad_area;
  area_ += quad_area;

  return true;
}

}  // namespace

namespace impeller {

std::shared_ptr<DlVerticesGeometry>
    ShadowTessellator::MakeAmbientShadowVertices(const PathSource& source,
                                                 Scalar occluder_height,
                                                 const Matrix& matrix) {
  PolygonInfo polygon(source, matrix);
  if (!polygon.IsValid()) {
    return nullptr;
  }

  return nullptr;
}


}  // namespace impeller
