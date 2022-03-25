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
 * @file ADRCRateControl.cpp
 */

#include <ADRCRateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void ADRCRateControl::setGains(const matrix::Vector3f &b, const matrix::Vector3f &bw_c, const matrix::Vector3f &bw_o){
	_gain_b = b;
	_bw_c = bw_c;
	_bw_o = bw_o;

	// TODO: Remove hardcoded sample time
	float Ts = 1.0f/250.0f; // Sample time
	SquareMatrix3f A;
	Vector3f B;
	Matrix<float, 1, 3> C;

	// Define prototypes
	A.setIdentity();
	A(0,1) = Ts;
	A(0,2) = (Ts*Ts)/2;
	A(1,2) = Ts;
	B(0) = (Ts*Ts)/2;
	B(1) = Ts;
	C(0,0) = 1.0;

	// Populate controller gain vectors
	_gains_roll = calculateControllerGains(bw_c(0));
	_gains_pitch = calculateControllerGains(bw_c(1));
	_gains_yaw = calculateControllerGains(bw_c(2));

	// Populate observer gain vectors
	_L_roll = calculateObserverGains(_bw_o(0), Ts);
	_L_pitch = calculateObserverGains(_bw_o(1), Ts);
	_L_yaw = calculateObserverGains(bw_o(2), Ts);

	// Populate observer matrices
	_zmat_roll = A - (_L_roll * C * A);
	_zmat_pitch = A - (_L_pitch * C * A);
	_zmat_yaw = A - (_L_yaw * C * A);

	_umat_roll = B - (_L_roll * C * B);
	_umat_pitch = B - (_L_pitch * C * B);
	_umat_yaw = B - (_L_yaw * C * B);

}

Vector3f ADRCRateControl::update(const Vector3f &rate, const Vector3f &rate_sp, bool enabled)
{

	// Run observers on current state and previous control input
	_z_roll = _zmat_roll*_z_roll + _umat_roll*prev_out(0) + _L_roll*rate(0);
	_z_pitch = _zmat_pitch*_z_pitch + _umat_pitch*prev_out(1) + _L_pitch*rate(1);
	_z_yaw = _zmat_yaw*_z_yaw + _umat_yaw*prev_out(2) + _L_yaw*rate(2);

	// Calculate control outputs
	Vector3f torque;
	torque(0) = (_gains_roll(0)*(rate_sp(0) - _z_roll(0)) - _gains_roll(1)*_z_roll(1) - _z_roll(2))/_gain_b(0);
	torque(1) = (_gains_pitch(0)*(rate_sp(1) - _z_pitch(0)) - _gains_pitch(1)*_z_pitch(1) - _z_pitch(2))/_gain_b(1);
	torque(2) = (_gains_yaw(0)*(rate_sp(2) - _z_yaw(0)) - _gains_yaw(1)*_z_yaw(1) - _z_yaw(2))/_gain_b(2);

	torque = saturateController(torque, -1.0f, 1.0f);
	publishLESOState(torque, rate, rate_sp, enabled);
	return torque;
}

Vector2f ADRCRateControl::calculateControllerGains(float bw_c)
{
	Vector2f gain;
	gain(0) = bw_c * bw_c;
	gain(1) = 2.0f * bw_c;

	return gain;
}

Vector3f ADRCRateControl::calculateObserverGains(float bw_o, float Ts)
{
	Vector3f L;

	float z_eso = expf(-bw_o*Ts); //Mapping pole in S plane to Z plane
	L(0) = 1.0f-powf(z_eso, 3.0f);
	L(1) = (3.0f/(2.0f*Ts)) * powf((1.0f - z_eso), 2.0f) * (1.0f + z_eso);
	L(2) = (1.0f/powf(Ts, 2.0f)) * powf((1.0f -z_eso), 3.0f);

	return L;
}

Vector3f ADRCRateControl::saturateController(Vector3f torque, float min, float max)
{
	Vector3f ret;
	for (int i = 0; i < 3; i++){
		ret(i) = math::min(math::max(torque(i), min),max);
	}
	return ret;
}

void ADRCRateControl::publishLESOState(Vector3f output, Vector3f measurements, Vector3f setpoint, bool enabled)
{
	adrc_leso_s leso_state = {};
	leso_state.timestamp = hrt_absolute_time();
	leso_state.r = setpoint(0);
	leso_state.y = measurements(0);
	leso_state.u = output(0);
	leso_state.z1 = _z_roll(0);
	leso_state.z2 = _z_roll(1);
	leso_state.z3 = _z_roll(2);
	leso_state.ctrl_en = enabled;
	leso_state.b = _gain_b(0);
	leso_state.bwc = _bw_c(0);
	leso_state.bwo = _bw_o(0);

	_leso_state_pub.publish(leso_state);
}
