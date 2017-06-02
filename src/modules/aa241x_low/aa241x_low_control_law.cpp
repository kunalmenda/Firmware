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
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>

using namespace aa241x_low;

const int num_waypoints = 4;
float waypoint_Ns [num_waypoints] = {-2550.0f, -2320.0f, -2500.0f, -2300.0f };
float waypoint_Es [num_waypoints] = {1840.0f, 1840.0f, 1900.0f, 1900.0f};
float waypoint_Bs [num_waypoints] = {10.0f,10.0f,10.0f,10.0f};

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */
void low_loop()
{

	// float my_float_variable = 0.0f;		/**< example float variable */

	// getting high data value example
	// float my_high_data = high_data.field1;

	// setting low data value example
    float init_time = high_data.field15;
    float elapsed_time_s = (hrt_absolute_time() - init_time)/1000000.0f;

if (elapsed_time_s  < 60.0f)  {
       float c_N = -2400.0f;
       float c_E = 1940.0f;
       float rho = 20.0f;
       float lambda = 1.0f;
       if (elapsed_time_s > 30.0f) {
           lambda = -1.0f;
       }

       low_data.field1 = c_N;
       low_data.field2 = c_E;
       low_data.field3 = rho;
       low_data.field4 = lambda;
       low_data.field5 = 2;
}
else{

	float dist_to_waypoint = 
		sqrtf(powf(waypoint_Ns[1] - position_N, 2) + 
			powf(waypoint_Es[1] - position_E, 2));

	if(dist_to_waypoint < waypoint_Bs[1]){
		float tmp_N = waypoint_Ns[0];
		float tmp_E = waypoint_Es[0];
		float tmp_B = waypoint_Bs[0];


		for(int i=1;i<num_waypoints;i++){
			waypoint_Ns[i-1] = waypoint_Ns[i];
			waypoint_Es[i-1] = waypoint_Es[i];
			waypoint_Bs[i-1] = waypoint_Bs[i];
		}
		waypoint_Ns[num_waypoints-1] = tmp_N;
		waypoint_Es[num_waypoints-1] = tmp_E;
		waypoint_Bs[num_waypoints-1] = tmp_B;
	}

	float r_N = waypoint_Ns[0];
	float r_E = waypoint_Es[0];
	float q_N = waypoint_Ns[1] - waypoint_Ns[0];
	float q_E = waypoint_Es[1] - waypoint_Es[0];
	float normq = sqrtf( powf(q_N,2) + powf(q_E,2) );
	q_E /= normq;
	q_N /= normq;

	low_data.field1 = r_N;
	low_data.field2 = r_E;
	low_data.field3 = q_N;
	low_data.field4 = q_E;
        low_data.field5 = 1;

	float psi_q = atan2f(q_E,q_N);
	float dist_from_anchor = 
		-sinf(psi_q) * (position_N - r_N) + cosf(psi_q) * (position_E - r_E);

	float seg_length = sqrtf( pow(waypoint_Ns[1]-r_N,2) + pow(waypoint_Es[1]-r_E,2) );

	if(dist_from_anchor > seg_length){ // overshot
		float tmp_N = waypoint_Ns[0];
		float tmp_E = waypoint_Es[0];
		float tmp_B = waypoint_Bs[0];


		for(int i=1;i<num_waypoints;i++){
			waypoint_Ns[i-1] = waypoint_Ns[i];
			waypoint_Es[i-1] = waypoint_Es[i];
			waypoint_Bs[i-1] = waypoint_Bs[i];
		}
		waypoint_Ns[num_waypoints-1] = tmp_N;
		waypoint_Es[num_waypoints-1] = tmp_E;
		waypoint_Bs[num_waypoints-1] = tmp_B;
	}
}

}
