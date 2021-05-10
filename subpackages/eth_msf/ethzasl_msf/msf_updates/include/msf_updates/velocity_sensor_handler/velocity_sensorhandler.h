/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef VELOCITY_SENSOR_H
#define VELOCITY_SENSOR_H

#include <queue>

#include <geometry_msgs/TwistStamped.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PointStamped.h>

#include <msf_updates/velocity_sensor_handler/velocity_measurement.h>

namespace msf_velocity_sensor {
class VelocitySensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Matrix<double, 3, 1> z_v_;   ///< velocity measurement.
  double n_zv_;  ///< velocity measurement noise.
  ros::Subscriber subVelocity_;
  void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
 public:
  VelocitySensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 3, 1> GetVelocityMeasurement() {
    return z_v_;
  }
  // Setters for configure values.
  void SetNoises(double n_zv);
};
}  // namespace msf_velocity_sensor
#include "implementation/velocity_sensorhandler.hpp"
#endif  // VELOCITY_SENSOR_H
