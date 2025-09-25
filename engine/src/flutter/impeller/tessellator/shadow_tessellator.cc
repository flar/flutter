// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/tessellator/shadow_tessellator.h"

#include "flutter/impeller/geometry/path_source.h"
#include "flutter/impeller/tessellator/path_tessellator.h"

namespace {

using impeller::PathTessellator;
using impeller::Point;
using impeller::Scalar;
using impeller::Vector2;

class PolygonInfo : impeller::PathTessellator::VertexWriter {
 public:
  PolygonInfo(const impeller::PathSource& path,
              const impeller::Matrix& matrix,
              Scalar occluder_height);

  bool IsValid() const { return is_valid_; }

 private:
  const Scalar shadow_size_;

  // Each point in the polygon form of the path is turned into a structure
  // that tracks the gradient of the shadow at that point in the path. The
  // shape is turned into a sort of pin cushion where each structure acts
  // like a pin pushed into that cushion in the direction of the shadow
  // gradient at that location.
  //
  // Each entry contains the direction of the pin at that location and the
  // depth to which the pin is inserted, expressed as a fraction of the full
  // umbra size specified by the shadow parameters. A depth of 1.0 means
  // the pin was inserted all the way to the depth of the shadow gradient
  // and didn't collide with any other pins. A fraction less than 1.0 can
  // occur if either the shape was too small and the pins intersected with
  // other pins across the shape from them, or if the curvature in a given
  // area was so tight that adjacent pins started bumping into their neighbors
  // even if the overall size of the shape was larger than the shadow.
  //
  // Different pins will be shortened by different amounts in the same shape
  // depending on the local geometry (tight curves or narrow cross section).
  struct UmbraPin {
    // The point on the original path that generated this entry into the
    // umbra geometry. AKA the point at which this pin was stabbed.
    Point path_vertex;

    // The unit vector pointing to the interior that defines the direction
    // of the shadow gradient at this point in the path.
    Vector2 pin_direction;

    // The interior penetration of the umbra starts out at the full blur
    // radius, but can be shortened when pins are too crowded and start
    // intersecting each other due to tight curvature or a narrow overall
    // shape cross section in this direction.
    //
    // Note that the penumbra (the part of the shadow that is outside the
    // path) will always be fully realized (penumbra_fraction == 1.0).
    Scalar umbra_fraction = 1.0f;
  };

  std::vector<UmbraPin> pins_;
  bool is_valid_ = true;  // aka single convex contour

  Point centroid_;
  Scalar shape_area_ = 0.0f;
  Scalar direction_ = 0.0f;
  bool path_ended_ = false;

  Scalar max_umbra_size_ = 0.0f;

  // |VertexWriter|
  void Write(Point point);

  // |VertexWriter|
  void EndContour();

  // Parameters that determine the sub-pixel grid we will use to simplify
  // the contours to avoid degenerate differences in the vertices.
  static constexpr Scalar kSubPixelCount = 16.0f;
  static constexpr Scalar kSubPixelScale = (1.0f / kSubPixelCount);

  static constexpr Scalar GetUmbraSizeForHeight(Scalar occluder_height) {
    return occluder_height;  // TODO(jimgraham): not really, compute this
  }

  // Transforms point and rounds to the sub-pixel grid.
  Point AdjustPoint(Point point);

  // Validates that the given point continues the path on a single contour,
  // non-self-intersecting, convex path and updates the area and centroid
  // point based on the partial areas of the new point, the previous point
  // and the first point.
  //
  // The method tests for multiple conditions of convexity. If the last 3
  // points are not turning the same direction as previous points, then the
  // path is locally not convex and therefore "invalid". If the quad area of
  // the last 2 points wrt the first point (computed for area and centroid)
  // is not the same sign as the contour's direction then the path is now
  // progressing the "wrong way" around the initial point which indicates
  // multiple turns and self-intersection and is thus "invalid".
  //
  // Note that the area is only computed in order to finalize the centroid
  // point at the end.
  bool ValidatePointAndUpdateCentroid(const Point& new_point);

  // Finalize the weighted centroid using the area calculated while the
  // path was being delivered.
  void FinalizeCentroid();

  // Run through the pins and determine the closest pin to the centroid
  // and, in particular, if it is less than the required umbra distance.
  void ComputePinDirectionsAndMinDistanceToCentroid();

  // Run through the path calculating the outset vertices for the penumbra
  // and connecting them to the inset vertices of the umbra and then to
  // the centroid in a system of triangles with the appropriate alpha values
  // representing the intensity of the (non-gamma-adjusted) shadow at those
  // points.
  void ComputeMesh();
};

PolygonInfo::PolygonInfo(const impeller::PathSource& source,
                         const impeller::Matrix& matrix,
                         Scalar occluder_height)
    : shadow_size_(GetUmbraSizeForHeight(occluder_height)),
      centroid_(0.0f, 0.0f),
      max_umbra_size_(shadow_size_) {
  Scalar scale = matrix.GetMaxBasisLengthXY();

  auto [point_count, contour_count] =
      impeller::PathTessellator::CountFillStorage(source, scale);
  pins_.reserve(point_count);

  PathTessellator::PathToTransformedFilledVertices(source, *this, matrix);
  if (!is_valid_ || pins_.size() < 3 || direction_ == 0 || shape_area_ == 0) {
    is_valid_ = false;
    return;
  }

  FinalizeCentroid();

  ComputePinDirectionsAndMinDistanceToCentroid();
}

void PolygonInfo::Write(Point point) {
  if (path_ended_) {
    is_valid_ = false;
    return;
  }

  if (!is_valid_) {
    return;
  }

  point = AdjustPoint(point);

  if (!pins_.empty()) {
    // If this isn't the first point then we need to perform de-duplication
    // and possibly convexity checking and centroid updates.

    if (point == pins_.back().path_vertex) {
      // Avoid duplicate points, adjusted points are rounded so == is OK
      // for floating point comparison here.
      return;
    }

    if (!ValidatePointAndUpdateCentroid(point)) {
      FML_DCHECK(!is_valid_);
      return;
    }
  }

  pins_.emplace_back(point);
}

void PolygonInfo::EndContour() {
  if (path_ended_) {
    is_valid_ = false;
    return;
  }

  // PathTessellator always ensures the path is closed back to the origin
  // by an extra call to Write(Point).
  FML_DCHECK(pins_.front().path_vertex == pins_.back().path_vertex);
  pins_.pop_back();
  path_ended_ = true;
}

Point PolygonInfo::AdjustPoint(Point point) {
  // Transform to device space and round to nearest sub-pixel.
  return (point * kSubPixelCount).Round() * kSubPixelScale;
}

bool PolygonInfo::ValidatePointAndUpdateCentroid(const Point& new_point) {
  if (!is_valid_) {
    return false;
  }

  if (pins_.size() < 2u) {
    return true;
  }

  const Point& prev = pins_.back().path_vertex;

  if (direction_ != 0) {
    // Note that if there are only 2 points in the path, the direction_
    // will not yet have been determined and this check computes the same
    // values as the global check below anyway.
    FML_DCHECK(pins_.size() > 2u);

    // The area convexity check below only checks if the contour goes around
    // once, but that contour could still contain locally concave points that
    // happen to continue their path around the initial point in the same
    // direction. (Consider a circle with a dent on the far side of it, the
    // points all progress in one direction from the perspective of the first
    // point even though the 3 vertices forming the dent are themselves
    // concave).

    const Point& prev_prev = pins_.end()[-2].path_vertex;
    Vector2 v0 = prev - prev_prev;
    Vector2 v1 = new_point - prev_prev;
    if (v0.Cross(v1) * direction_ < 0) {
      return is_valid_ = false;
    }
  }

  const Point& first = pins_.front().path_vertex;
  Vector2 v0 = prev - first;
  Vector2 v1 = new_point - first;
  Scalar quad_area = v0.Cross(v1);

  // convexity check for whole path which can detect if we turn more than
  // 360 degrees and start going the other way wrt the start point, but
  // does not detect if any pair of points are concave (checked above).
  if (direction_ == 0) {
    direction_ = std::copysign(1.0f, quad_area);
  } else if (quad_area * direction_ < 0) {
    return is_valid_ = false;
  }

  // We are computing the centroid using a weighted average of all of the
  // centroids of the triangles in a tessellation of the polygon, in this
  // case a triangle fan tessellation relative to the first point in the
  // polygon.  We could use any point, but since we had to compute the
  // cross product above relative to the initial point in order to detect
  // if the path turned more than once, we already have values available
  // relative to that first point here.
  //
  // The centroid of each triangle is the 3-way average of the corners of
  // that triangle. Since the triangles are all relative to the first point,
  // one of those corners is (0, 0) in this relative triangle and so we
  // can simple add up the x,y of the two relative points and divide by
  // 3.0. Since all values in the sum are divided by 3.0, we can save that
  // constant division until the end when we finalize the average computation.
  //
  // We also weight these centroids by the area of the triangle so that we
  // adjust for the parts of the polygon that are represented more densely
  // and the parts that span a larger part of its circumference. A simple
  // average would bias the centroid towards parts of the polygon where the
  // points are denser. If we are rendering a polygonal representation of a
  // round rect with only one round corner, all of the many approximating
  // segments of the flattened round corner would overwhelm the handful of
  // other simple segments for the flat sides. A weighted average places the
  // centroid back at the "center of mass" of the polygon.
  //
  // Luckily, the same cross product used above that helps us determine the
  // turning and convexity of the polygon also provides us with the area of
  // the parallelogram projected from the 3 points in the triangle. That
  // area is exactly double the area of the triangle itself. We could divide
  // by 2 here, but since we are also accumulating these cross product values
  // for the final weighted division, the factors of 2 all cancel out.
  //
  // quad_area is (2 * triangle area).
  // centroid_ is accumulating sum(3 * triangle centroid * quad area).
  // shape_area_ is accumulating sum(quad area).
  //
  // The final combined average weight factor will be (3 * sum(quad area)).
  centroid_ += (v0 + v1) * quad_area;
  shape_area_ += quad_area;

  return true;
}

void PolygonInfo::FinalizeCentroid() {
  // To finalize the centroid value we need to divide by both the constant
  // factor of 3.0 that was present when we computed the triangle centroids
  // and also the accumulated weight from the triangle area factor.
  centroid_ /= 3.0f * shape_area_;

  // The centroid accumulation was relative to the first point in the
  // polygon so we make it absolute here.
  centroid_ += pins_[0].path_vertex;
}

void PolygonInfo::ComputePinDirectionsAndMinDistanceToCentroid() {
  Scalar min_umbra_squared = shadow_size_ * shadow_size_;
  FML_DCHECK(direction_ == 1.0f || direction_ == -1.0f);

  // For simplicity of iteration, we start with the last vertex as the
  // "previous" pin and then iterate once over the vector of pins,
  // performing these calculations on the path segment from the previous
  // pin to the current pin. In the end, all pins and therefore all path
  // segments are processed once even if we start with the last pin.
  UmbraPin& prev_pin = pins_.back();
  for (UmbraPin& cur_pin : pins_) {
    // We point the pin in towards the center of the shape perpendicular
    // to the segment that follows the (previous) path point.
    prev_pin.pin_direction = (cur_pin.path_vertex - prev_pin.path_vertex)
                                 .Normalize()
                                 .PerpendicularRight() *
                             direction_;

    // Accumulate (min) the distance from the centroid to "this" segment.
    Scalar distance_squared = centroid_.GetDistanceToSegmentSquared(
        cur_pin.path_vertex, prev_pin.path_vertex);
    min_umbra_squared = std::min(min_umbra_squared, distance_squared);

    prev_pin = cur_pin;
  }

  max_umbra_size_ = std::sqrt(min_umbra_squared);
}

}  // namespace

namespace impeller {

std::shared_ptr<ShadowTessellator> ShadowTessellator::MakeAmbientShadowVertices(
    const PathSource& source,
    Scalar occluder_height,
    const Matrix& matrix) {
  PolygonInfo polygon(source, matrix, occluder_height);
  if (!polygon.IsValid()) {
    return nullptr;
  }

  return nullptr;
}

}  // namespace impeller
