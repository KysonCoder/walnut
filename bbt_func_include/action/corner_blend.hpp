/**
 * @file corner_blend.hpp
 * @brief Corner blend geometry calculator
 * @author Claude
 * @version 0.1
 * @date 2026-05-27
 *
 * Self-contained corner blend computation. Given three targets (blend_start,
 * corner_point, blend_end), computes the blended path through the corner zone.
 *
 * This class was extracted from PathGenerator::BlendCartesianSegments to live
 * in the action library so that each CAction can compute its own corner data
 * independently.
 */

#ifndef __CORNER_BLEND_HPP__
#define __CORNER_BLEND_HPP__

#include "target.hpp"
#include "actiondata.hpp"
#include "quaternion.hpp"

#include <vector>

/**
 * @brief Corner blend calculator
 *
 * Encapsulates the corner-blending algorithm (parabolic + sine weighting)
 * originally from PathGenerator::BlendCartesianSegments.
 */
class CornerBlend {
public:
    // Inputs (set by preprocessor)
    CTarget blend_start;               ///< Start of blend on previous segment
    CTarget corner_point;              ///< Corner center (original target)
    CTarget blend_end;                 ///< End of blend on next segment

    double corner_radius_m{0.01};      ///< Corner zone radius (m)
    double sample_step_m{0.001};       ///< Sampling step (m)

    // Outputs (filled by Compute())
    std::vector<PathInterpolationPoint> blend_points;  ///< Interpolated points
    double blend_length{0.0};          ///< Total corner path length

    /**
     * @brief Compute the blended corner path
     *
     * Uses parabolic + sine weighting (k=0.5) from the original
     * BlendCartesianSegments implementation.
     *
     * @return true if successful, false if degenerate (zero radius, collinear, etc.)
     */
    bool Compute();

private:
    /**
     * @brief Blend two orientations with sine weighting
     */
    math::Quaternion<double> BlendOrientation(
        const math::Quaternion<double>& q1,
        const math::Quaternion<double>& q2,
        double w1) const;
};

#endif // __CORNER_BLEND_HPP__
