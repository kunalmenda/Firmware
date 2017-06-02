/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/*
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float altitude_desired = 0.0f;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {

	// float my_float_variable = 0.0f;		/**< example float variable */


	// // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
	// if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop, 
	// 																 //	should only occur on first engagement since this is 59Hz loop
	// 	// yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
	// 	// altitude_desired = position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
        // 	high_data.field14 = position_N;
        // 	high_data.field15 = position_E;
        // 	high_data.field16 = yaw;
	// }	
	// An example of how to run a one time 'setup' for example to lock one's altitude and heading...
	if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop, 
																	 //	should only occur on first engagement since this is 59Hz loop
		// yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
		// altitude_desired = position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
        high_data.field14 = position_D_gps;
        high_data.field15 = hrt_absolute_time();
        //high_data.field15 = yaw;
        //high_data.field16 = position_N * sinf(yaw) - position_E * cosf(yaw);  // line offset parameter
	}	


	// TODO: write all of your flight control here...

         float r_N = low_data.field1; // c_N if orbit
         float r_E = low_data.field2; // c_E if orbit
         float q_N = low_data.field3; // rho if orbit
         float q_E = low_data.field4; // lambda if orbit
         int mode = low_data.field5;


	// extract high params
        //float h_command = aah_parameters.h_command; // altitude
	float u_command = aah_parameters.u_command; // velocity
        //float psi_command = aah_parameters.psi_command; // heading

	float k_u = aah_parameters.k_u; // throttle gain
	float k_h = aah_parameters.k_h; // altitude gain
	float k_theta = aah_parameters.k_theta; // theta gain
	float k_phi = aah_parameters.k_phi; // phi gain
	float k_psi = aah_parameters.k_psi; // psi gain
	float k_y = aah_parameters.k_y; // line follow gain
        float k_circ = aah_parameters.k_circ;
	float throt_trim = aah_parameters.throt_trim;
        float psi_inf = aah_parameters.psi_inf;



	// float beta = asin(speed_body_v / speed_body_u);

	// Line following logic
    float anchor_h = high_data.field14;
    //float anchor_psi = high_data.field15;
    //float anchor_rho = high_data.field16;



	// Elevator and Throttle control
	float throttle = k_u * (u_command - speed_body_u) + throt_trim;
    // float theta_command = k_h * (h_command - position_D_gps);
    float theta_command = k_h * (anchor_h - position_D_gps);
	theta_command = std::min( std::max( theta_command, -(float)0.35 ), (float)0.35 );
	float elevator = k_theta * (theta_command - pitch);

	// Rudder and Roll
	float rudder = 0.0;
	float pi = (float)M_PI;
    // line following:

    //float dist = anchor_rho + position_E * cosf(anchor_psi) - position_N * sinf(anchor_psi);
        float psi_command = 0;
        float dist = 0;
        if (mode == 2) {
            float pos_angle = atan2f(position_E-r_E,position_N-r_N);
            dist = sqrtf( powf(position_N-r_N,2) + powf(position_E-r_E,2) );
            psi_command = pos_angle + q_E*(pi/(float)2.0 + atanf(k_circ*(dist-q_N)/q_N));
        }

        else {
        float psi_q = atan2f(q_E, q_N);
        dist =
                -sinf(psi_q) * (position_N - r_N) + cosf(psi_q) * (position_E - r_E);

        psi_command = psi_inf*atanf(k_y * dist) + psi_q;
        }

	float psi_diff = psi_command - yaw;

	if (psi_diff > pi) {
		psi_diff -= 2.0f*pi;
}
	if (psi_diff < -pi) {
		psi_diff += 2.0f*pi;
}
	float phi_command = k_psi * psi_diff;
        phi_command = std::min( std::max( phi_command, -0.79f), 0.79f);
	float aileron = k_phi * (phi_command - roll);
	// if(psi_diff < -0.0001f || psi_diff > 0.0001f){
	// 	rudder += -1.0f;
	// }
	// float aileron = k_phi * ((float)0. - roll); // will just attempt 0 roll. Change when needed


	// Saturation
	elevator = std::min(std::max((double)elevator,-1.0),1.0);
	aileron = std::min(std::max((double)aileron,-1.0),1.0);
	rudder = std::min(std::max((double)rudder,-1.0),1.0);
	throttle = std::min(std::max((double)throttle,0.0),1.0);

	// Set output of roll servo to the control law output calculated above
	roll_servo_out = aileron;		
	// as an example, just passing through manual control to everything but roll
	pitch_servo_out = elevator;
	yaw_servo_out = rudder;
	throttle_servo_out = throttle;


	// setting high data value example
	// high_data.variable_name1 = my_float_variable;
	// field1 through 16
	high_data.field1 = elevator;
	high_data.field2 = aileron;
	high_data.field3 = rudder;
	high_data.field4 = throttle;
	// inputs
    high_data.field5 = u_command;
    high_data.field6 = dist;
	// gains
	high_data.field7 = k_y; 
	high_data.field8 = k_u;
	high_data.field9 = k_h;
	high_data.field10 = k_theta;
	high_data.field11 = k_phi;
	high_data.field12 = k_psi;
	// trims
	high_data.field13 = throt_trim;

	// fields 14-16 reserved for line follow}

}
