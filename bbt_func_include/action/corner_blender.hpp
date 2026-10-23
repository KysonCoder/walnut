/**
 * @file corner_blender.hpp
 * @brief Parabolic+sine corner blender for waypoint sequences (pre-sampling)
 */

#ifndef CORNER_BLENDER_HPP
#define CORNER_BLENDER_HPP

#include <deque>
#include <vector>

#include "target.hpp"

namespace action {

/**
 * @brief Replaces sharp interior corners with parabolic+sine blend curves.
 *
 * Operates on a sequence of CTarget waypoints.  For each interior waypoint
 * the parabolic+sine blend curve (k=0.5, same weights as the legacy
 * CornerBlendData formula) is pre-sampled between the blend start (on the
 * incoming segment) and blend end (on the outgoing segment).  The corner
 * point itself is NOT emitted (except for degenerate corners), so the output
 * polyline never revisits it — this eliminates the 158° direction reversal
 * observed when sampled paths revisited the corner point.
 *
 * Every output waypoint owns a distinct TargetData in @p output_storage
 * (std::deque keeps addresses stable across push_back).  The caller must keep
 * @p output_storage alive as long as @p output is used.  Generated curve
 * waypoints have is_joints_valid = false so that pipeline Step0 resolves
 * their joints via IK.
 */
class CornerBlender {
 public:
  CornerBlender() = default;

  /**
   * @brief Blend corners in a waypoint sequence.
   *
   * @param[in] input Input waypoint sequence (at least 3 waypoints for blending).
   * @param[in] blend_radius_m Requested blend radius in meters.
   * @param[in] sample_step_m Arc-length sampling step for blend curves [m].
   * @param[out] output Blended waypoint sequence (forward order).
   * @param[out] output_storage Backing TargetData for output waypoints.
   * @return true if blending was performed, false if input had fewer than 3 waypoints.
   */
  bool Blend(const std::vector<CTarget>& input,
             double blend_radius_m,
             double sample_step_m,
             std::vector<CTarget>& output,
             std::deque<TargetData>& output_storage);
};

}  // namespace action

#endif  // CORNER_BLENDER_HPP
