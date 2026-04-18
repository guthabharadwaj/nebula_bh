// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEBULA_OUSTER_XYZ_LUT_HPP
#define NEBULA_OUSTER_XYZ_LUT_HPP

#include "nebula_ouster_decoders/ouster_metadata.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace nebula::drivers
{

/// @brief Precomputed per-beam / per-column trig tables for XYZ conversion.
/// @details Given a range measurement r at (beam, measurement_id), the Cartesian coordinates are:
///   encoder = -2π * measurement_id / columns_per_frame          (Ouster convention — CW rotation)
///   theta   = encoder + beam_azimuth_offset[beam]
///   phi     = beam_altitude[beam]
///   r_m     = range_mm * 1e-3
///   x = r_m * cos(phi) * cos(theta) + beam_origin * cos(theta)
///   y = r_m * cos(phi) * sin(theta) + beam_origin * sin(theta)
///   z = r_m * sin(phi)
/// The beam_origin offset corresponds to lidar_origin_to_beam_origin_mm (converted to meters) and
/// accounts for the horizontal offset of the beam optical center from the sensor frame origin.
class OusterXyzLut
{
public:
  /// @param metadata Parsed sensor metadata. Requires valid beam angles and columns_per_frame.
  explicit OusterXyzLut(const OusterMetadata & metadata)
  : columns_per_frame_(metadata.columns_per_frame),
    pixels_per_column_(metadata.pixels_per_column),
    beam_origin_m_(metadata.lidar_origin_to_beam_origin_mm * 1e-3),
    pixel_shift_(metadata.pixel_shift_by_row)
  {
    constexpr double k_deg_to_rad = M_PI / 180.0;
    beam_cos_elev_.resize(pixels_per_column_);
    beam_sin_elev_.resize(pixels_per_column_);
    beam_azimuth_rad_.resize(pixels_per_column_);
    beam_cos_az_.resize(pixels_per_column_);
    beam_sin_az_.resize(pixels_per_column_);
    for (size_t b = 0; b < pixels_per_column_; ++b) {
      const double elev_rad = metadata.beam_altitude_angles_deg[b] * k_deg_to_rad;
      beam_cos_elev_[b] = std::cos(elev_rad);
      beam_sin_elev_[b] = std::sin(elev_rad);
      beam_azimuth_rad_[b] = metadata.beam_azimuth_angles_deg[b] * k_deg_to_rad;
      beam_cos_az_[b] = std::cos(beam_azimuth_rad_[b]);
      beam_sin_az_[b] = std::sin(beam_azimuth_rad_[b]);
    }

    cos_encoder_.resize(columns_per_frame_);
    sin_encoder_.resize(columns_per_frame_);
    const double k_two_pi = 2.0 * M_PI;
    for (size_t c = 0; c < columns_per_frame_; ++c) {
      // Negative sign: Ouster measurement_id increases with clockwise rotation, but ROS right-hand
      // coordinate frame expects counter-clockwise azimuth.
      const double encoder = -k_two_pi * static_cast<double>(c) /
                             static_cast<double>(columns_per_frame_);
      cos_encoder_[c] = std::cos(encoder);
      sin_encoder_[c] = std::sin(encoder);
    }
  }

  /// @brief Compute sensor-frame XYZ for a single measurement.
  /// @param range_mm Raw range value in millimeters.
  /// @param beam_idx Beam (row) index in [0, pixels_per_column).
  /// @param measurement_id Column index in [0, columns_per_frame).
  /// @param[out] x, y, z Point in sensor frame, meters.
  /// @param[out] azimuth_rad, elevation_rad Spherical coordinates of the measured direction.
  inline void compute(
    uint32_t range_mm, size_t beam_idx, size_t measurement_id, float & x, float & y, float & z,
    float & azimuth_rad, float & elevation_rad) const
  {
    // NOTE: The beam_azimuth_angles field in the metadata already carries the per-beam angular
    // offset. Do NOT also apply pixel_shift_by_row here — that field is a visualization hint
    // (how to re-grid a scan into a staggered 2D image) and applying it would double-count the
    // correction.
    const double r_m = static_cast<double>(range_mm) * 1e-3;

    const double cos_e = cos_encoder_[measurement_id];
    const double sin_e = sin_encoder_[measurement_id];
    const double cos_off = beam_cos_az_[beam_idx];
    const double sin_off = beam_sin_az_[beam_idx];
    const double cos_theta = cos_e * cos_off - sin_e * sin_off;
    const double sin_theta = sin_e * cos_off + cos_e * sin_off;

    const double cos_phi = beam_cos_elev_[beam_idx];
    const double sin_phi = beam_sin_elev_[beam_idx];

    const double xy = r_m * cos_phi + beam_origin_m_;
    x = static_cast<float>(xy * cos_theta);
    y = static_cast<float>(xy * sin_theta);
    z = static_cast<float>(r_m * sin_phi);

    azimuth_rad = static_cast<float>(std::atan2(sin_theta, cos_theta));
    elevation_rad = static_cast<float>(std::asin(sin_phi));
  }

  [[nodiscard]] size_t columns_per_frame() const { return columns_per_frame_; }
  [[nodiscard]] size_t pixels_per_column() const { return pixels_per_column_; }

private:
  size_t columns_per_frame_;
  size_t pixels_per_column_;
  double beam_origin_m_;
  std::vector<int32_t> pixel_shift_;
  std::vector<double> beam_cos_elev_;
  std::vector<double> beam_sin_elev_;
  std::vector<double> beam_azimuth_rad_;
  std::vector<double> beam_cos_az_;
  std::vector<double> beam_sin_az_;
  std::vector<double> cos_encoder_;
  std::vector<double> sin_encoder_;
};

}  // namespace nebula::drivers

#endif  // NEBULA_OUSTER_XYZ_LUT_HPP
