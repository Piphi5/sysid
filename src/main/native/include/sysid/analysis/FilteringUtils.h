// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "sysid/analysis/AnalysisManager.h"

namespace sysid {

/**
 * Trims quasistatic data so that no point has a voltage of zero or a velocity
 * less than the motion threshold.
 *
 * @tparam S        The size of the raw data array.
 * @tparam Voltage  The index of the voltage entry in the raw data.
 * @tparam Velocity The index of the velocity entry in the raw data.
 *
 * @param data            A pointer to the vector of raw data.
 * @param motionThreshold The velocity threshold under which to delete data.
 */
template <size_t S, size_t Voltage, size_t Velocity>
void TrimQuasistaticData(std::vector<std::array<double, S>>* data,
                         double motionThreshold) {
  data->erase(std::remove_if(data->begin(), data->end(),
                             [motionThreshold](const auto& pt) {
                               return std::abs(pt[Voltage]) <= 0 ||
                                      std::abs(pt[Velocity]) < motionThreshold;
                             }),
              data->end());
}

/**
 * Reduces noise in velocity data by applying a median filter.
 *
 * @tparam S The size of the raw data array
 * @tparam Velocity The index of the velocity entry in the raw data.
 *
 * @param data the prepared data vector containing acceleration data
 * @param window the size of the window of the median filter (must be odd)
 */
template <size_t S, size_t Velocity>
std::vector<std::array<double, S>> ApplyMedianFilter(
    const std::vector<std::array<double, S>>& data, int window) {
  if (window % 2 == 0) {
    throw std::runtime_error("Window must be an odd number");
  }

  size_t step = window / 2;

  std::vector<std::array<double, S>> prepared;

  // Run median filter on non-edges
  for (int i = step; i < data.size() - step; i++) {
    std::vector<std::array<double, S>> velocity_window(window);
    velocity_window.assign(data.begin() + i - step,
                           data.begin() + i + step + 1);
    std::sort(velocity_window.begin(), velocity_window.end(),
              [&](std::array<double, S> x, std::array<double, S> y) {
                return x[Velocity] < y[Velocity];
              });
    prepared.push_back(std::array<double, S>{data[i]});
    prepared.back()[Velocity] = velocity_window[step][Velocity];
  }

  return prepared;
}

/**
 * Trims the step voltage data to discard all points before the maximum
 * acceleration and after reaching stead-state velocity.
 *
 * @param data A pointer to the step voltage data.
 * @param stepTestDuration A reference to the step test duration that will be
 * used.
 * @param minStepTime A reference to the minimum step time that will be used.
 */
inline void TrimStepVoltageData(std::vector<PreparedData>* data,
                                float& stepTestDuration, double& minStepTime) {
  double firstTimestamp = data->at(0).timestamp;

  // Trim data before max acceleration
  data->erase(data->begin(),
              std::max_element(
                  data->begin(), data->end(), [](const auto& a, const auto& b) {
                    return std::abs(a.acceleration) < std::abs(b.acceleration);
                  }));

  minStepTime = std::min(data->at(0).timestamp - firstTimestamp, minStepTime);

  // If step duration hasn't been set yet, set calculate a default (find the
  // entry before the acceleration first hits zero)
  if (static_cast<double>(stepTestDuration) <= minStepTime) {
    // Find latest element with nonzero acceleration
    auto endIt = std::find_if(std::reverse_iterator{data->end()},
                              std::reverse_iterator{data->begin()},
                              [&](const PreparedData& entry) {
                                return std::abs(entry.acceleration) > 0.0;
                              });

    // Calculate default duration
    stepTestDuration = static_cast<float>(
        endIt->timestamp - data->front().timestamp + minStepTime);
  }

  // Find first entry greater than the step test duration
  auto maxIt =
      std::find_if(data->begin(), data->end(), [&](PreparedData entry) {
        return entry.timestamp - data->front().timestamp + minStepTime >
               stepTestDuration;
      });

  // Trim data beyond desired step test duration
  if (maxIt != data->end()) {
    data->erase(maxIt, data->end());
  }
}
}  // namespace sysid
