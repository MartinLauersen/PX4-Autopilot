/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ADRCRateControl.hpp
 *
 * ADRC 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>

class ADRCRateControl
{
public:
	ADRCRateControl() = default;
	~ADRCRateControl() = default;

	/**
	 * Set the rate control gains
	 * @param b 3D vector of feedback gains for body x,y,z axis
	 * @param bw_c 3D vector of controller bandwidth
	 * @param bw_o 3D vector of observer bandwidth
	 */
	void setGains(const matrix::Vector3f &b, const matrix::Vector3f &bw_c, const matrix::Vector3f &bw_o);

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);


private:

	// Gains
	matrix::Vector3f _gain_b;  ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _bw_c;    ///< rate control controller bandwidth
	matrix::Vector3f _bw_o;    ///< rate control observer bandwidth

	// States
	matrix::Vector3f _roll_z;  ///< Roll states
	matrix::Vector3f _pitch_z; ///< Pitch states
	matrix::Vector3f _yaw_z;   ///< Yaw states

	// Observer Matrices & Vecotrs
	matrix::Vector3f _roll_L;     ///< Roll observer vector
	matrix::Matrix3f _roll_zmat;  ///< Roll observer matrix for z
	matrix::Matrix3f _roll_umat;  ///< Roll observer matrix for u
	matrix::Vector3f _pitch_L;    ///< Pitch observer vector
	matrix::Matrix3f _pitch_zmat; ///< Pitch observer matrix for z
	matrix::Matrix3f _pitch_umat; ///< Pitch observer matrix for u
	matrix::Vector3f _yaw_L;      ///< Yaw observer vector
	matrix::Matrix3f _yaw_zmat;  ///< Yaw observer matrix for z
	matrix::Matrix3f _yaw_umat;  ///< Yaw observer matrix for u

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;
};
