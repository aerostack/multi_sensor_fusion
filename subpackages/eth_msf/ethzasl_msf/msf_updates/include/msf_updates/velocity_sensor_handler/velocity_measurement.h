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
#ifndef VELOCITY_MEASUREMENT_HPP_
#define VELOCITY_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <tf_conversions/tf_eigen.h>

namespace velocity_measurement {
enum {
    nMeasurements = 3
};
/**
 * \brief A measurement as provided by a velocity sensor.
 */
typedef msf_core::MSF_Measurement<geometry_msgs::PointStamped,
Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> VelocityMeasurementBase;
struct VelocityMeasurement : public VelocityMeasurementBase {
private:
    typedef VelocityMeasurementBase Measurement_t;
    typedef Measurement_t::Measurement_ptr measptr_t;

    virtual void MakeFromSensorReadingImpl(measptr_t msg) {
        Eigen::Matrix<double, nMeasurements,
                msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
        Eigen::Matrix<double, nMeasurements, 1> r_old;

        H_old.setZero();

        // Get measurements.
        z_v_(0) = msg->point.x; // vx
        z_v_(1) = msg->point.y; // vy
        z_v_(2) = msg->point.z;

        const double s_zv = n_zv_ * n_zv_;
        R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zv).finished()
                .asDiagonal();
    }
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, 3, 1> z_v_;  /// Velocity measurement.
    double n_zv_;  /// Velocity measurement noise.

    typedef msf_updates::EKFState EKFState_T;
    typedef EKFState_T::StateDefinition_T StateDefinition_T;
    virtual ~VelocityMeasurement() {}
    VelocityMeasurement(double n_zv, bool isabsoluteMeasurement, int sensorID,
                        bool enable_mah_outlier_rejection, double mah_threshold)
        : VelocityMeasurementBase(isabsoluteMeasurement, sensorID,
                                  enable_mah_outlier_rejection, mah_threshold),
          n_zv_(n_zv) {}
    virtual std::string Type() { return "velocity"; }
    /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
    virtual void Apply(shared_ptr<EKFState_T> non_const_state,
                       msf_core::MSF_Core<EKFState_T>& core) {
        // Init variables.
        Eigen::Matrix<double, nMeasurements,
                msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
        Eigen::Matrix<double, nMeasurements, 1> r_old;

        H_old.setZero();

        if (non_const_state->time == msf_core::constants::INVALID_TIME) {
            MSF_WARN_STREAM(
                        "Apply velocity update was called with an invalid state.");
            return;  // Early abort.
        }

        const EKFState_T& state = *non_const_state;

        enum {
            idx_v = msf_tmp::GetStartIndex<EKFState_T::StateSequence_T,
            typename msf_tmp::GetEnumStateType<EKFState_T::StateSequence_T,
            StateDefinition_T::v>::value,
            msf_tmp::CorrectionStateLengthForType>::value,
        };

        // Construct H matrix.
        // Vx
        H_old.block<1, 1>(0, idx_v)(0) = 1;  // v_x
        // Vy
        H_old.block<1, 1>(1, idx_v + 1)(0) = 1;  // v_y
        //Vz
        H_old.block<1, 1>(2, idx_v + 2)(0) = 1;  // v_z


        //std::cout << "state V " << state.Get<StateDefinition_T::v>() << std::endl;

        // Construct residuals
        r_old.block<1, 1>(0, 0)(0) = z_v_(0) - state.Get<StateDefinition_T::v>().block<1, 1>(0, 0)(0);
        r_old.block<1, 1>(1, 0)(0) = z_v_(1) - state.Get<StateDefinition_T::v>().block<1, 1>(1, 0)(0);
        r_old.block<1, 1>(2, 0)(0) = z_v_(2) - state.Get<StateDefinition_T::v>().block<1, 1>(2, 0)(0);


        // Call update step in base class.
        this->CalculateAndApplyCorrection(non_const_state, core, H_old, r_old, R_);
    }
};
}  // namespace velocity_measurement
#endif  // VELOCITY_MEASUREMENT_HPP_
