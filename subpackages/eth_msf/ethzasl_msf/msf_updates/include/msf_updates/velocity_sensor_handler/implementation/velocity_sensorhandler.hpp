/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#include <msf_core/eigen_utils.h>
#ifndef VELOCITY_SENSORHANDLER_HPP_
#define VELOCITY_SENSORHANDLER_HPP_

namespace msf_velocity_sensor {
VelocitySensorHandler::VelocitySensorHandler(
        msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
        std::string topic_namespace, std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zv_(1e-6) {
    ros::NodeHandle pnh("~/velocity_sensor");
    ros::NodeHandle nh("msf_updates");

    pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
    pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

    subVelocity_ =
            nh.subscribe<geometry_msgs::PointStamped>
            ("velocity", 20, &VelocitySensorHandler::MeasurementCallback, this);

    //memset(heightbuff, 0, sizeof(double) * heightbuffsize);

}

void VelocitySensorHandler::SetNoises(double n_zv) {
    n_zv_ = n_zv;
}

void VelocitySensorHandler::MeasurementCallback(
        const geometry_msgs::PointStampedConstPtr & msg) {

    received_first_measurement_ = true;

    this->SequenceWatchDog(msg->header.seq, subVelocity_.getTopic());
    MSF_INFO_STREAM_ONCE(
                "*** velocity sensor got first measurement from topic "
                << this->topic_namespace_ << "/" << subVelocity_.getTopic()
                << " ***");

    shared_ptr<velocity_measurement::VelocityMeasurement> meas(
                new velocity_measurement::VelocityMeasurement(
                    n_zv_, true, this->sensorID, enable_mah_outlier_rejection_,
                    mah_threshold_));
    meas->MakeFromSensorReading(msg, msg->header.stamp.toSec());

    z_v_ = meas->z_v_;  // Store this for the init procedure.

    this->manager_.msf_core_->AddMeasurement(meas);
}
}  // namespace msf_velocity_sensor
#endif  // VELOCITY_SENSORHANDLER_HPP_
