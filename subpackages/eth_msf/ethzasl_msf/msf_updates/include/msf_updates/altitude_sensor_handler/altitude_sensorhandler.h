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
#ifndef ALTITUDE_SENSOR_H

#include <queue>

#include <geometry_msgs/PointStamped.h>
#include <msf_core/msf_sensormanagerROS.h>

#include <msf_updates/altitude_sensor_handler/altitude_measurement.h>

namespace msf_altitude_sensor {
class AltitudeSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  Eigen::Matrix<double, 1, 1> z_p_;  ///< Pressure measurement.
  double n_zp_;  ///< Pressure measurement noise.
  ros::Subscriber subPressure_;
  void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
 public:
  AltitudeSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 1, 1> GetPressureMeasurement() {
    return z_p_;
  }
  // Setters for configure values.
  void SetNoises(double n_zp);
};
}  // namespace msf_altitude_sensor
#include "implementation/altitude_sensorhandler.hpp"
#endif  // POSE_SENSOR_H
