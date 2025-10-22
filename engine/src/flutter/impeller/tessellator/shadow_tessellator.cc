// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/tessellator/shadow_tessellator.h"

#include "flutter/impeller/geometry/path_source.h"
#include "flutter/impeller/tessellator/path_tessellator.h"

namespace {

using impeller::Color;
using impeller::kEhCloseEnough;
using impeller::Matrix;
using impeller::PathTessellator;
using impeller::Point;
using impeller::Scalar;
using impeller::ScalarNearlyZero;
using impeller::ShadowVertices;
using impeller::Tessellator;
using impeller::Vector2;

class PolygonInfo : impeller::PathTessellator::VertexWriter {
 public:
  static constexpr Scalar GetUmbraSizeForHeight(Scalar occluder_height) {
    return occluder_height;  // TODO(jimgraham): not really, compute this
  }

  PolygonInfo(const impeller::PathSource& path,
              const impeller::Matrix& matrix,
              Scalar shadow_size,
              const Tessellator::Trigs& trigs);

  bool IsValid() const { return is_valid_; }
  bool IsEmpty() const { return is_empty_; }

  std::shared_ptr<ShadowVertices> TakeVertices() {
    if (!is_valid_) {
      return nullptr;
    }

    return std::make_shared<ShadowVertices>(std::move(vertices_),  //
                                            std::move(indices_),   //
                                            std::move(colors_));
  }

 private:
  // The natural size of the shadow (both umbra and penumbra) computed from
  // the height of the occluder. For the penumbra, this size will be used.
  // For the umbra, its size may be reduced if the shape is much smaller
  // than the natural shadow size. See |umbra_size_|.
  const Scalar shadow_size_;

  // Each point in the polygon form of the path is turned into a structure
  // that tracks the gradient of the shadow at that point in the path. The
  // shape is turned into a sort of pin cushion where each struct acts
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
    static constexpr Scalar kFractionUninitialized = -1.0f;

    // The point on the original path that generated this entry into the
    // umbra geometry.
    //
    // AKA the point on the path at which this pin was stabbed.
    Point path_vertex;

    // The vector from this path segment to the next.
    Vector2 path_delta;

    // The location of the head of the pin (the part outside the shape).
    Vector2 pin_delta;

    // The location of the end of this pin, taking into account the reduction
    // of the umbra_size due to minimum distance to centroid, but ignoring
    // clipping against other pins.
    Point pin_tip;

    // The location that this pin confers to the umbra polygon. Initially,
    // this is the same as the pin_tip, but can be reduced by intersecting
    // and clipping against other pins.
    Point umbra_vertex;

    // The interior penetration of the umbra starts out at the full blur
    // radius as modified by the global distance of the path segments to
    // the centroid, but can be shortened when pins are too crowded and start
    // intersecting each other due to tight curvature.
    Scalar umbra_fraction = kFractionUninitialized;

    // Used to create a circular linked list while pruning the umbra polygon.
    // The final vertices that are in the umbra polygon are the vertices that
    // remain on this linked list from a "head" pin.
    UmbraPin* pNext = nullptr;
    UmbraPin* pPrev = nullptr;

    bool IsFractionInitialized() const {
      return umbra_fraction > kFractionUninitialized;
    }
  };

  std::vector<UmbraPin> pins_;
  UmbraPin* umbra_vertices_head_ = nullptr;

  bool is_valid_ = true;   // aka single convex contour
  bool is_empty_ = false;  // is there anything to cast a shadow?

  Point centroid_;
  Scalar shape_area_ = 0.0f;
  Scalar direction_ = 0.0f;
  bool path_ended_ = false;

  // The distance that the umbra extends inside the shape, computed from the
  // (min of the) distances from every segment of the outline to the centroid.
  Scalar umbra_size_ = 0.0f;

  // The vertex mesh result that represents the shadow, to be rendered
  // using a modified indexed variant of DrawVertices that also adjusts
  // the alpha of the colors on a per-pixel basis by mapping their linear
  // alphas into a gaussian curve.
  const impeller::Tessellator::Trigs& trigs_;
  std::vector<Point> vertices_;
  std::vector<uint16_t> indices_;
  std::vector<Color> colors_;

  // |VertexWriter|
  void Write(Point point);

  // |VertexWriter|
  void EndContour();

  // Parameters that determine the sub-pixel grid we will use to simplify
  // the contours to avoid degenerate differences in the vertices.
  static constexpr Scalar kSubPixelCount = 16.0f;
  static constexpr Scalar kSubPixelScale = (1.0f / kSubPixelCount);

  // Rounds the device coordinate to the sub-pixel grid.
  Point ToDeviceGrid(Point point);

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

  // Run through the pins and determine if they intersect each other
  // internally, whether they are completely obscured by other pins,
  // their new relative lengths if they defer to another pin at some
  // depth, and which remaining pins are part of the umbra polygon.
  void ResolveUmbraIntersections();

  // Structure to store the result of computing the intersection between
  // 2 pins, containing the point of intersection and the relative fractions
  // at which the pins intersected (expressed as a ratio of 0 to 1 where
  // 0 represents intersecting at the path outline and 1 represents
  // intersecting at the tip of the pin where the umbra is darkest.
  struct PinIntersection {
    Point intersection;
    Scalar fraction0;
    Scalar fraction1;
  };

  static constexpr Scalar kCrossTolerance = 1.0f / 2048.0f;
  static constexpr Scalar kIntersectionTolerance = 1.0e-6f;

  static constexpr Scalar FiniteVectorLengthSquared(Vector2 v) {
    return !v.IsFinite() ? -1.0f : v.Dot(v);
  }

  static constexpr inline bool OutsideInterval(Scalar numer,
                                               Scalar denom,
                                               bool denomPositive) {
    return (denomPositive && (numer < 0 || numer > denom)) ||
           (!denomPositive && (numer > 0 || numer < denom));
  }

  // Return the intersection between the 2 pins pPin0 and pPin1 if there
  // is an intersection.
  static std::optional<PinIntersection> ComputeIntersection(UmbraPin* pPin0,
                                                            UmbraPin* pPin1);

  static void RemovePin(UmbraPin* pPin, UmbraPin** pHead);

  static int ComputeSide(const Point& p0, const Vector2& v, const Point& p);

  // Run through the path calculating the outset vertices for the penumbra
  // and connecting them to the inset vertices of the umbra and then to
  // the centroid in a system of triangles with the appropriate alpha values
  // representing the intensity of the (non-gamma-adjusted) shadow at those
  // points.
  void ComputeMesh();

  const UmbraPin* FindBestInset(const UmbraPin* prev,
                                const UmbraPin* next,
                                const UmbraPin* p_cur_inner_pin);

  uint16_t AppendFan(const Point& center,
                     const Point& fan_start,
                     const Point& fan_end,
                     uint16_t center_index,
                     uint16_t prev_index);

  uint16_t AppendVertex(const Point& vertex, Scalar opacity);

  void AddTriangle(uint16_t v0, uint16_t v1, uint16_t v2);
};

PolygonInfo::PolygonInfo(const impeller::PathSource& source,
                         const Matrix& matrix,
                         Scalar shadow_size,
                         const Tessellator::Trigs& trigs)
    : shadow_size_(shadow_size),
      centroid_(0.0f, 0.0f),
      umbra_size_(shadow_size_),
      trigs_(trigs) {
  Scalar scale = matrix.GetMaxBasisLengthXY();

  auto [point_count, contour_count] =
      impeller::PathTessellator::CountFillStorage(source, scale);
  pins_.reserve(point_count);

  PathTessellator::PathToTransformedFilledVertices(source, *this, matrix);
  if (!is_valid_ || pins_.size() < 3 || direction_ == 0.0f ||
      shape_area_ == 0.0f) {
    // The shape was either invalid or empty.
    return;
  }

  FinalizeCentroid();

  ComputePinDirectionsAndMinDistanceToCentroid();

  ResolveUmbraIntersections();

  ComputeMesh();
}

// Enter a new point for the polygon approximation of the shape. Points are
// normalized to a device subpixel grid based on |kSubPixelCount|, duplicates
// at that sub-pixel grid are ignored, and the centroid is updated from
// the remaining non-duplicate grid points.
void PolygonInfo::Write(Point point) {
  if (path_ended_) {
    is_valid_ = false;
    return;
  }

  if (!is_valid_) {
    return;
  }

  point = ToDeviceGrid(point);

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

// Called at the end of every contour of which we hope there is only one.
// If we detect more than one contour then the shadow tessellation becomes
// invalid.
//
// Each contour will have exactly one point at the beginning and end which
// are duplicates. The extra repeat of the first point actually helped the
// centroid accumulation do its math for ever segment in the path, but
// going forward we don't need the extra pin in the shape so we verify that
// it is a duplicate and then we delete it.
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

// Adjust the device point to its nearest sub-pixel grid location.
Point PolygonInfo::ToDeviceGrid(Point point) {
  // Transform to device space and round to nearest sub-pixel.
  return (point * kSubPixelCount).Round() * kSubPixelScale;
}

// This method performs 3 functions.
// - Ensure that the 3 most recent vertices are turning in a consistent
//   direction. (No concave sections.)
// - Ensure that all vertices turn the same direction from the perspective
//   of the first point. (No "going around twice".)
// - Accumulate data to compute the centroid
bool PolygonInfo::ValidatePointAndUpdateCentroid(const Point& new_point) {
  if (!is_valid_) {
    return false;
  }

  if (pins_.size() < 2u) {
    return true;
  }

  const Point& prev = pins_.back().path_vertex;

  // direction_ is always normalized to one of these values.
  FML_DCHECK(direction_ == 0.0f ||  //
             direction_ == 1.0f ||  //
             direction_ == -1.0f);
  // We can only perform concavity detection once we have a direction.
  if (direction_ != 0.0f) {
    // Note that if there are only 2 points in the path, the direction_
    // will not yet have been determined and this check computes the same
    // values as the global check below anyway.
    FML_DCHECK(pins_.size() > 2u);

    // Check that each triplet of points (this, prev, prev_prev) turn the
    // same direction.
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
  // factor of 3.0 that was ignored when we computed the triangle centroids
  // (average of the 3 triangle vertices) and also the accumulation of all
  // of the individual triangle areas used as the averaging weights.
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

  // First pass, compute the smallest distance to the centroid.
  UmbraPin* p_prev_pin = &pins_.back();
  for (UmbraPin& pin : pins_) {
    UmbraPin* p_curr_pin = &pin;

    // Accumulate (min) the distance from the centroid to "this" segment.
    Scalar distance_squared = centroid_.GetDistanceToSegmentSquared(
        p_prev_pin->path_vertex, p_curr_pin->path_vertex);
    min_umbra_squared = std::min(min_umbra_squared, distance_squared);

    p_prev_pin = p_curr_pin;
  }

  umbra_size_ = std::sqrt(min_umbra_squared);

  // Second pass, fill out the pin data with the final umbra size.
  //
  // We also link all of the pins into a circular linked list so they can be
  // quickly eliminated in the method that resolves intersections of the pins.
  p_prev_pin = &pins_.back();
  for (UmbraPin& pin : pins_) {
    UmbraPin* p_curr_pin = &pin;
    p_curr_pin->pPrev = p_prev_pin;
    p_prev_pin->pNext = p_curr_pin;

    // We compute the vector along the path segment from the previous
    // path vertex to this one as well as the unit direction vector
    // that points from that pin towards the center of the shape,
    // perpendicular to that segment.
    p_prev_pin->path_delta = p_curr_pin->path_vertex - p_prev_pin->path_vertex;
    Vector2 pin_direction = p_prev_pin
                                ->path_delta  //
                                .Normalize()
                                .PerpendicularRight() *
                            direction_;

    p_prev_pin->pin_delta = pin_direction * umbra_size_;
    p_prev_pin->pin_tip = p_prev_pin->path_vertex + p_prev_pin->pin_delta;
    p_prev_pin->umbra_vertex = p_prev_pin->pin_tip;

    p_prev_pin = p_curr_pin;
  }
}

// Compute the intersection 'p' between the two pins pin 0 and pin 1, if any.
// The intersection structure will contain the fractional distances along the
// pins of the intersection and the intersection point itself if there is an
// intersection.
//
// The intersection structure will be reset to empty otherwise.
//
// This method was converted nearly verbatim from the Skia source files
// SkShadowTessellator.cpp and SkPolyUtils.cpp, except for variable
// naming and differences in the methods on Point and Vertex2.
std::optional<PolygonInfo::PinIntersection> PolygonInfo::ComputeIntersection(
    UmbraPin* pPin0,
    UmbraPin* pPin1) {
  Vector2 v0 = pPin0->path_delta;
  Vector2 v1 = pPin1->path_delta;
  Vector2 tip_delta = pPin1->pin_tip - pPin0->pin_tip;
  Vector2 w = tip_delta;
  Scalar denom = pPin0->path_delta.Cross(pPin1->path_delta);
  bool denomPositive = (denom > 0);
  Scalar sNumer, tNumer;
  if (ScalarNearlyZero(denom, kCrossTolerance)) {
    // segments are parallel, but not collinear
    if (!ScalarNearlyZero(tip_delta.Cross(pPin0->path_delta),
                          kCrossTolerance) ||
        !ScalarNearlyZero(tip_delta.Cross(pPin1->path_delta),
                          kCrossTolerance)) {
      return std::nullopt;
    }

    // Check for zero-length segments
    Scalar v0dotv0 = FiniteVectorLengthSquared(v0);
    if (v0dotv0 <= 0.0f) {
      // Both are zero-length
      Scalar v1dotv1 = FiniteVectorLengthSquared(v1);
      if (v1dotv1 <= 0.0f) {
        // Check if they're the same point
        if (w.IsFinite() && !w.IsZero()) {
          // *p = s0.fP0;
          // *s = 0;
          // *t = 0;
          return {{
              .intersection = pPin0->pin_tip,
              .fraction0 = 0.0f,
              .fraction1 = 0.0f,
          }};
        } else {
          // Intersection is indeterminate
          return std::nullopt;
        }
      }
      // Otherwise project segment0's origin onto segment1
      tNumer = v1.Dot(-w);
      denom = v1dotv1;
      if (OutsideInterval(tNumer, denom, true)) {
        return std::nullopt;
      }
      sNumer = 0;
    } else {
      // Project segment1's endpoints onto segment0
      sNumer = v0.Dot(w);
      denom = v0dotv0;
      tNumer = 0;
      if (OutsideInterval(sNumer, denom, true)) {
        // The first endpoint doesn't lie on segment0
        // If segment1 is degenerate, then there's no collision
        Scalar v1dotv1 = FiniteVectorLengthSquared(v1);
        if (v1dotv1 <= 0.0f) {
          return std::nullopt;
        }

        // Otherwise try the other one
        Scalar oldSNumer = sNumer;
        sNumer = v0.Dot(w + v1);
        tNumer = denom;
        if (OutsideInterval(sNumer, denom, true)) {
          // it's possible that segment1's interval surrounds segment0
          // this is false if params have the same signs, and in that case
          // no collision
          if (sNumer * oldSNumer > 0) {
            return std::nullopt;
          }
          // otherwise project segment0's endpoint onto segment1 instead
          sNumer = 0;
          tNumer = v1.Dot(-w);
          denom = v1dotv1;
        }
      }
    }
  } else {
    sNumer = w.Cross(v1);
    if (OutsideInterval(sNumer, denom, denomPositive)) {
      return std::nullopt;
    }
    tNumer = w.Cross(v0);
    if (OutsideInterval(tNumer, denom, denomPositive)) {
      return std::nullopt;
    }
  }

  Scalar localS = sNumer / denom;
  Scalar localT = tNumer / denom;

  return {{
      .intersection = pPin0->pin_tip + v0 * localS,
      .fraction0 = localS,
      .fraction1 = localT,
  }};
}

void PolygonInfo::RemovePin(UmbraPin* pPin, UmbraPin** pHead) {
  UmbraPin* pNext = pPin->pNext;
  UmbraPin* pPrev = pPin->pPrev;
  pPrev->pNext = pNext;
  pNext->pPrev = pPrev;
  if (*pHead == pPin) {
    *pHead = (pNext == pPin) ? nullptr : pNext;
  }
}

// Computes the relative direction for point p compared to segment defined
// by origin p0 and vector v. A positive value means the point is to the
// left of the segment, negative is to the right, 0 is collinear.
int PolygonInfo::ComputeSide(const Point& p0,
                             const Vector2& v,
                             const Point& p) {
  Vector2 w = p - p0;
  Scalar cross = v.Cross(w);
  if (!impeller::ScalarNearlyZero(cross, kCrossTolerance)) {
    return ((cross > 0) ? 1 : -1);
  }

  return 0;
}

// This method was converted nearly verbatim from the Skia source files
// SkShadowTessellator.cpp and SkPolyUtils.cpp, except for variable
// naming and differences in the methods on Point and Vertex2.
void PolygonInfo::ResolveUmbraIntersections() {
  UmbraPin* p_head_pin = &pins_.front();
  UmbraPin* p_curr_pin = p_head_pin;
  UmbraPin* p_prev_pin = p_curr_pin->pPrev;
  size_t umbra_vertex_count = pins_.size();

  // we should check each edge against each other edge at most once
  size_t allowed_iterations = pins_.size() * pins_.size() + 1u;

  while (p_head_pin && p_prev_pin != p_curr_pin) {
    if (--allowed_iterations == 0) {
      is_valid_ = false;
      return;
    }

    std::optional<PinIntersection> intersection =
        ComputeIntersection(p_prev_pin, p_curr_pin);
    if (intersection.has_value()) {
      // If the new intersection is further back on previous inset from the
      // prior intersection...
      if (intersection->fraction0 < p_prev_pin->umbra_fraction) {
        // no point in considering this one again
        RemovePin(p_prev_pin, &p_head_pin);
        --umbra_vertex_count;
        // go back one segment
        p_prev_pin = p_prev_pin->pPrev;
      } else if (p_curr_pin->IsFractionInitialized() &&
                 p_curr_pin->umbra_vertex.GetDistanceSquared(
                     intersection->intersection) < kIntersectionTolerance) {
        // We've already considered this intersection and come to the same
        // result, we're done.
        break;
      } else {
        // Add intersection.
        p_curr_pin->umbra_vertex = intersection->intersection;
        p_curr_pin->umbra_fraction = intersection->fraction1;

        // go to next segment
        p_prev_pin = p_curr_pin;
        p_curr_pin = p_curr_pin->pNext;
      }
    } else {
      // if previous pin is to right side of the current pin...
      int side = direction_ * ComputeSide(p_curr_pin->pin_tip,     //
                                          p_curr_pin->path_delta,  //
                                          p_prev_pin->pin_tip);
      if (side < 0 &&
          side == direction_ * ComputeSide(p_curr_pin->pin_tip,     //
                                           p_curr_pin->path_delta,  //
                                           p_prev_pin->pin_tip +
                                               p_prev_pin->path_delta)) {
        // no point in considering this one again
        RemovePin(p_prev_pin, &p_head_pin);
        --umbra_vertex_count;
        // go back one segment
        p_prev_pin = p_prev_pin->pPrev;
      } else {
        // move to next segment
        RemovePin(p_curr_pin, &p_head_pin);
        --umbra_vertex_count;
        p_curr_pin = p_curr_pin->pNext;
      }
    }
  }

  if (!p_head_pin) {
    is_valid_ = false;
    return;
  }

  // The head pin is automatically included as the first point of the umbra
  // polygon.
  p_prev_pin = p_head_pin;
  p_curr_pin = p_head_pin->pNext;
  size_t umbra_vertices = 1u;
  while (p_curr_pin != p_head_pin) {
    if (p_prev_pin->umbra_vertex.GetDistanceSquared(p_curr_pin->umbra_vertex) <
        kSubPixelScale * kSubPixelScale) {
      RemovePin(p_curr_pin, &p_head_pin);
      p_curr_pin = p_curr_pin->pNext;
    } else {
      umbra_vertices++;
      p_prev_pin = p_curr_pin;
      p_curr_pin = p_curr_pin->pNext;
    }
    FML_DCHECK(p_curr_pin == p_prev_pin->pNext);
    FML_DCHECK(p_prev_pin == p_curr_pin->pPrev);
  }

  if (umbra_vertices >= 3u) {
    umbra_vertices_head_ = p_head_pin;
  } else {
    is_valid_ = false;
  }
}

// The mesh computed connects all of the points in two rings. The outermost
// ring represents the point where the shadow disappears and those points
// are associated with an alpha of 0. The umbra polygon represents the ring
// where the shadow is its darkest, usually fully "opaque" (potentially
// modulated by a non-opaque shadow color, but opaque with respect to the
// shadow's varying intensity). The umbra polygon may not be fully "opaque"
// with respect to the shadow cast by the shape if the shadows radius is
// larger than the cross-section of the shape. If the umbra polygon is pulled
// back from extending the shadow distance inward due to this phenomenon,
// then the umbra_color will be computed to be less than fully opaque.
//
// The mesh will connect the centroid to the umbra (inner) polygon at a
// constant level as computed in umbra_color, and then the umbra polygon
// is connected to the nearest points on the penumbra (outer) polygon which
// is seeded with points that are fully transparent (umbra level 0).
//
// This creates 2 rings of triangles that are interspersed in the vertices_
// and connected into triangles using indices_ both to reuse the vertices
// as best we can and also because we don't generate the vertices in any
// kind of useful fan or strip format. The points are reused as such:
//
// - The centroid vertex will be used once for each pair of umbra vertices
//   to make triangles for the inner ring.
// - Each umbra vertex will be used in both the inner and the outer rings.
//   In particular, in 2 of the inner ring triangles and in an arbitrary
//   number of the outer ring vertices (each outer ring vertex is connected
//   to the neariest inner ring vertex so the mapping is not predictable).
// - Each outer ring vertex is used in at least 2 outer ring triangles, the
//   one that links to the vertex before it and the one that links to the
//   vertex following it, plus we insert extra vertices on the outer ring
//   to turn the corners beteween the projected segments.
void PolygonInfo::ComputeMesh() {
  if (!is_valid_ || !umbra_vertices_head_) {
    FML_LOG(ERROR) << "is_valid: " << is_valid_ << "umbra_vertices: " << umbra_vertices_head_;
    is_valid_ = false;
    return;
  }

  Scalar umbra_opacity = umbra_size_ / shadow_size_;
  AppendVertex(centroid_, umbra_opacity);

  const UmbraPin* p_inner_point = nullptr;
  uint16_t umbra_index = 0u;

  UmbraPin* p_prev_pin = &pins_.back();
  uint16_t penumbra_index = AppendVertex(
      p_prev_pin->path_vertex - p_prev_pin->pin_delta, 0.0f);

  // We now run through the list of all pins and append points and triangles
  // to our internal vectors.
  //
  // Points are appended for the penumbra polygon which is running across
  // the heads of all of our pins, as well as any intermediate points we
  // insert to round the corners between pins.
  //
  // Points are also appended for the umbra polygon which includes all of
  // the points that survived the process of insetting the polygon by the
  // interior umbra size and then resolving conflicts between pins.
  //
  // Indices are appended whenever we insert a new point to make triangles
  // both from the centroid to adjacent points in the umbra polygon and from
  // the (nearest) points on the umbra polygon to the many points on the
  // outer penumbra. We always insert 3 new indices rather than using a fan
  // format because not all triangles fan out from the same point.
  for (UmbraPin& pin : pins_) {
    UmbraPin* p_curr_pin = &pin;
    // First make sure we are basing our new penumbra triangles off of
    // the best choice of the inner umbra point.
    const UmbraPin* p_new_inner_point =
        FindBestInset(p_prev_pin, p_curr_pin, p_inner_point);

    if (p_new_inner_point == nullptr) {
      // We failed to match the umbra polygon to the outer polygon.
      FML_LOG(ERROR) << "FindBestInset failed";
      is_valid_ = false;
      return;
    }

    if (p_new_inner_point != p_inner_point) {
      // We have a new inner umbra point, we need to add it to the list
      // of vertices and, when we have more than one, make a triangle to
      // fill in the inner-most darkest part of the umbra.
      uint16_t new_umbra_index =
          AppendVertex(p_new_inner_point->umbra_vertex, umbra_opacity);

      // Make a triangle with the most recent pair of umbra indices (if we
      // have more than one) and the centroid (which is always at index 0).
      if (p_inner_point != nullptr) {
        FML_DCHECK(umbra_index != 0u);
        AddTriangle(0u, umbra_index, new_umbra_index);
      } else {
        FML_DCHECK(umbra_index == 0u);
      }

      // Update the new "current/most recent" umbra data.
      umbra_index = new_umbra_index;
      p_inner_point = p_new_inner_point;
    }

    // Now round the corner from the
    penumbra_index = AppendFan(p_inner_point->umbra_vertex,  //
                               p_prev_pin->path_vertex + p_prev_pin->pin_delta,
                               p_prev_pin->path_vertex + p_curr_pin->pin_delta,
                               umbra_index, penumbra_index);
    p_prev_pin = p_curr_pin;
  }
}

const PolygonInfo::UmbraPin* PolygonInfo::FindBestInset(
    const UmbraPin* p_prev,
    const UmbraPin* p_next,
    const UmbraPin* p_current_inner_pin) {
  if (p_current_inner_pin == nullptr) {
    // When pruning the list of pins to make the umbra polygon, the head
    // pointer was only ever moved forward through the list. So, the very
    // first path point we process should be "at or before" the first umbra
    // pin. If the head umbra pin was moved forward fairly far, then its
    // previous surviving umbra vertex might be closer, so we start there
    // and let the code below bump the pin forward if the distances suggest
    // it.
    p_current_inner_pin = umbra_vertices_head_->pPrev;
  }

  Scalar curr_distance_squared =
      p_current_inner_pin->umbra_vertex.GetDistanceSquared(p_prev->path_vertex);
  UmbraPin* p_next_inner_pin = p_current_inner_pin->pNext;
  Scalar next_distance_squared =
      p_next_inner_pin->umbra_vertex.GetDistanceSquared(p_prev->path_vertex);

  return (curr_distance_squared > next_distance_squared)
      ? p_next_inner_pin
      : p_current_inner_pin;
}

// Appends a fan based on center from the relative point in start_delta to
// the relative point in end_delta, potentially adding additional relative
// vectors if the turning rate is faster than the trig values in trigs_.
uint16_t PolygonInfo::AppendFan(const Point& center,
                                const Vector2& start_delta,
                                const Vector2& end_delta,
                                uint16_t center_index,
                                uint16_t prev_index) {
  for (auto trig : trigs_) {
    Point fan_delta = trig * start_delta;
    if (fan_delta.Cross(end_delta) * direction_ >= 0) {
      break;
    }
    uint16_t cur_index =
        AppendVertex(center + fan_delta, 0.0f);
    AddTriangle(center_index, prev_index, cur_index);
    prev_index = cur_index;
  }
  uint16_t cur_index =
      AppendVertex(center + end_delta, 0.0f);
  AddTriangle(center_index, prev_index, cur_index);
  return cur_index;
}

// Appends a vertex and color into the associated std::vectors and returns
// the index at which the point was inserted.
uint16_t PolygonInfo::AppendVertex(const Point& vertex, Scalar opacity) {
  FML_DCHECK(opacity >= 0.0f && opacity <= 1.0f);
  uint16_t index = vertices_.size();
  FML_DCHECK(index == colors_.size());
  // TODO(jimgraham): Turn this condition into a failure of the tessellation
  FML_DCHECK(index <= std::numeric_limits<uint16_t>::max());
  vertices_.push_back(vertex);
  colors_.emplace_back(0.0f, 0.0f, 0.0f, opacity);
  return index;
}

// Appends a triangle of the 3 indices into the indices_ vector.
void PolygonInfo::AddTriangle(uint16_t v0, uint16_t v1, uint16_t v2) {
  FML_DCHECK(std::max(std::max(v0, v1), v2) < vertices_.size());
  indices_.push_back(v0);
  indices_.push_back(v1);
  indices_.push_back(v2);
}

}  // namespace

namespace impeller {

std::shared_ptr<ShadowVertices> ShadowTessellator::MakeAmbientShadowVertices(
    Tessellator& tessellator,
    const PathSource& source,
    Scalar occluder_height,
    const Matrix& matrix) {
  Scalar umbra_size = PolygonInfo::GetUmbraSizeForHeight(occluder_height);
  PolygonInfo polygon(source, matrix, umbra_size,
                      tessellator.GetTrigsForDeviceRadius(umbra_size));

  return polygon.TakeVertices();
}

}  // namespace impeller
