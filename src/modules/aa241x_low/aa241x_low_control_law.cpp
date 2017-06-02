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

int P_ind = 2;
int follow_state = 1;
bool follow_done = false;

waypoint P[5];

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

        if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
            P[0].xy = makeTwoDvec(position_N, position_E);
            P[1].xy = makeTwoDvec(-2300.0f, 1780.0f);
            P[2].xy = makeTwoDvec(-2450.0f, 1860.0f);
            P[3].xy = makeTwoDvec(-2200.0f, 1840.0f);
            P[4].xy = makeTwoDvec(-2250.0f, 1650.0f);

            P[0].heading = yaw;
            twoDvec diff;
            for(int i=1; i<4; i++){
                diff = subVecs2D( P[i+1].xy , P[i-1].xy );
                P[i].heading = atan2f(diff.y,diff.x);
            }
            diff = subVecs2D(P[4].xy, P[3].xy);
            P[4].heading = atan2f(diff.y,diff.x);

            P_ind = 2;
            follow_state = 1;

        }



        float R = 15;

        dubinsParams dbParams;
        findDubinsParams(P[P_ind-1],P[P_ind],R,&dbParams);

        twoDvec pos = makeTwoDvec(position_N, position_E);

        waypointParams wpParams;
        followWaypointsDubins(dbParams, pos, R, &follow_state, &follow_done, &wpParams);

        if(follow_done){
            follow_done = false;
            P_ind++;
            if(P_ind > 5){
                P_ind = 2;
                P[0].xy = makeTwoDvec(position_N, position_E);
                P[0].heading = yaw;
                follow_state = 1;
            }

        }



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

twoDvec makeTwoDvec(float x, float y){
        twoDvec out;
        out.x = x;
        out.y = y;
        return out;
}

twoDvec mulMxVec2D(Matrix Mx, twoDvec twoDV){
        twoDvec outVec;
        outVec.x = Mx.x11 * twoDV.x + Mx.x12 * twoDV.y;
        outVec.y = Mx.x21 * twoDV.x + Mx.x22 * twoDV.y;

        return outVec;
}

twoDvec addVecs2D(twoDvec v1, twoDvec v2){
        twoDvec out;
        out.x = v1.x + v2.x;
        out.y = v1.y + v2.y;
        return out;
}

twoDvec scale2Dvec(float scalar, twoDvec inp){
        twoDvec out;
        out.x = scalar * inp.x;
        out.y = scalar * inp.y;
        return out;
}

twoDvec subVecs2D(twoDvec v1, twoDvec v2){
        return addVecs2D(v1, scale2Dvec(-1.0f, v2)  );
}

float norm2D(twoDvec vec){
        return sqrtf(powf(vec.x,2) + powf(vec.y,2));
}


float brk(float input){
        float output = fmod(input, 2.0f * PI);
        return output;
}

Matrix R_z(float v){
        Matrix out;
        out.x11 = cosf(v);
        out.x12 = -sinf(v);
        out.x21 = sinf(v);
        out.x22 =  cosf(v);
        return out;
}

void findDubinsParams( waypoint S, waypoint E, float R, dubinsParams* output ){
        twoDvec ps = S.xy;
        float psis = S.heading;

        twoDvec pe = E.xy;
        float psie = E.heading;

        twoDvec crs = addVecs2D(ps,
                scale2Dvec(R,
                        mulMxVec2D(R_z(PI/2.0f),
                                makeTwoDvec(cosf(psis),sinf(psis)) )));


        twoDvec cls = addVecs2D(ps,
                scale2Dvec(R,
                        mulMxVec2D(R_z(-PI/2.0f),
                                makeTwoDvec(cosf(psis),sinf(psis) ) )));

        twoDvec cre = addVecs2D(pe,
                scale2Dvec(R,
                        mulMxVec2D(R_z(PI/2.0f),
                                makeTwoDvec(cosf(psie),sinf(psie) ) )));

        twoDvec cle = addVecs2D(pe,
                scale2Dvec(R,
                        mulMxVec2D(R_z(-PI/2.0f),
                                makeTwoDvec(cosf(psie),sinf(psie) ) )));

        // Compute L1
        twoDvec line = subVecs2D(cre, crs);
        float v = atan2f(line.y, line.x);
        float L1 = norm2D(line) +
                R * brk(2.0f*PI - brk(v - PI/2.0f) - brk(psis - PI/2.0f)) +
                R * brk(2.0f*PI + brk(psie - PI/2.0f) - brk(v - PI/2.0f));


        // Compute L2
        line = subVecs2D(cle, crs);
        v = atan2f(line.y, line.x);
        float l = norm2D(line);
        float v2 = v - PI/2.0f + asinf(2.0f*R/l);
        float L2 = sqrtf(powf(l,2) - 4.0f*powf(R,2)) +
                R * brk(2.0f*PI + brk(v2) - brk(psis - PI/2.0f)) +
                R * brk(2.0f*PI + brk(v2 + PI) - brk(psie + PI/2.0f));


        // Compute L3
        line = subVecs2D(cre, cls);
        v = atan2f(line.y, line.x);
        l = norm2D(line);
        v2 = acosf(2.0f*R/l);
        float L3 = sqrtf(powf(l,2) - 4.0f*powf(R,2)) +
                R * brk(2.0f*PI + brk(psis + PI/2.0f) - brk(v+v2)) +
                R * brk(2.0f*PI + brk(psie - PI/2.0f) - brk(v+v2 -  PI));


        // Compute L4
        line = subVecs2D(cle, cls);
        v = atan2f(line.y, line.x);
        l = norm2D(line);
        float L4 = l +
                R * brk(2.0f*PI + brk(psis + PI/2.0f) - brk(v + PI/2.0f)) +
                R * brk(2.0f*PI + brk(v + PI/2.0f) - brk(psie + PI/2.0f));

        float L = L1;
        int L_ind = 1;
        if(L2 < L){
                L = L2;
                L_ind = 2;
        }
        if(L3 < L){
                L = L3;
                L_ind = 3;
        }
        if(L4 < L){
                L = L4;
                L_ind = 4;
        }

        output->L = L;

        twoDvec e1 = makeTwoDvec(1.0f,0.0f);

        twoDvec cs, ce;

        switch(L_ind){
                case 1:
                        output->cs = crs;
                output->lams = 1;
                output->ce = cre;
                cs = output->cs;
                ce = output->ce;
                line = subVecs2D(ce,cs);
                        v = atan2f(line.y, line.x);
                        l = norm2D(line);
                output->lame = 1;
                output->q1 = scale2Dvec( 1.0f/l , line);
                output->z1 = addVecs2D(cs,
                        scale2Dvec(R,
                                mulMxVec2D(R_z(-PI/2),output->q1)));
                output->z2 = addVecs2D(ce,
                        scale2Dvec(R,
                                mulMxVec2D(R_z(-PI/2),output->q1)));
                        break;
                case 2:
                        output->cs = crs;
                output->lams = 1;
                output->ce = cle;
                output->lame = -1;
                cs = output->cs;
                ce = output->ce;
                line = subVecs2D(ce,cs);
                        v = atan2f(line.y, line.x);
                        l = norm2D(line);
                v2 = v - PI/2.0f + asinf(2.0f*R/l);
                output->q1 = mulMxVec2D(R_z(v2 + PI/2.0f), e1);
                output->z1 = addVecs2D(cs, scale2Dvec( R, mulMxVec2D( R_z(v2), e1) ));
                output->z2 = addVecs2D( ce, scale2Dvec( R, mulMxVec2D( R_z(v2+PI), e1) ));
                        break;
                case 3:
                        output->cs = cls;
                output->lams = -1;
                output->ce = cre;
                output->lame = 1;
                cs = output->cs;
                ce = output->ce;
                line = subVecs2D(ce,cs);
                        v = atan2f(line.y, line.x);
                        l = norm2D(line);
                v2 = acosf(2.0f*R/l);
                output->q1 = mulMxVec2D( R_z(v+v2-PI/2), e1);
                output->z1 = addVecs2D(cs, scale2Dvec(R, mulMxVec2D(R_z(v+v2),e1)));
                output->z2 = addVecs2D(ce, scale2Dvec(R, mulMxVec2D(R_z(v+v2-PI),e1)));
                        break;
                case 4:
                        output->cs = cls;
                output->lams = -1;
                output->ce = cle;
                output->lame = -1;
                cs = output->cs;
                ce = output->ce;
                line = subVecs2D(ce,cs);
                        v = atan2f(line.y, line.x);
                        l = norm2D(line);
                output->q1 = scale2Dvec(1.0f/l, line);
                output->z1 = addVecs2D(cs, scale2Dvec(R, mulMxVec2D(R_z(PI/2.0f),output->q1)));
                output->z2 = addVecs2D(ce, scale2Dvec(R, mulMxVec2D(R_z(PI/2.0f),output->q1)));
                        break;
                default: break;
        }

        output->z3.x = pe.x;
        output->z3.y = pe.y;
        output->q3 = mulMxVec2D(R_z(psie), e1);
}

bool inHalfspace(twoDvec p, twoDvec z, twoDvec q){
        twoDvec pmz = subVecs2D(p,z);
        return (pmz.x * q.x + pmz.y * q.y > 0.0f);
}

void followWaypointsDubins(dubinsParams dbparams, twoDvec p, float R, int* statePtr, bool* done, waypointParams* wpParams){

        twoDvec cs = dbparams.cs;
        int lams = dbparams.lams;
        twoDvec ce = dbparams.ce;
        int lame = dbparams.lame;
        twoDvec z1 = dbparams.z1;
        twoDvec q1 = dbparams.q1;
        twoDvec z2 = dbparams.z2;
        twoDvec z3 = dbparams.z3;
        twoDvec q3 = dbparams.q3;

        int state = *statePtr;

        switch(state){
                case 1:
                        wpParams->flag = 2;
                        wpParams->c = cs;
                        wpParams->rho = R;
                        wpParams->lam = lams;
                        if(inHalfspace(p,z1,scale2Dvec(-1.0f,q1))){
                                *statePtr = 2;
                        }
                        break;
                case 2:
                        wpParams->flag = 2;
                        wpParams->c = cs;
                        wpParams->rho = R;
                        wpParams->lam = lams;
                        if(inHalfspace(p,z1,q1)){
                                *statePtr = 3;
                        }
                        break;
                case 3:
                        wpParams->flag = 1;
                        wpParams->r = z1;
                        wpParams->q = q1;
                        if(inHalfspace(p,z2,q1)){
                                *statePtr = 4;
                        }
                        break;
                case 4:
                        wpParams->flag = 2;
                        wpParams->c = ce;
                        wpParams->rho = R;
                        wpParams->lam = lame;
                        if(inHalfspace(p,z3, scale2Dvec(-1.0f, q3) )){
                                *statePtr = 5;
                        }
                        break;
                case 5:
                        wpParams->flag = 2;
                        wpParams->c = ce;
                        wpParams->rho = R;
                        wpParams->lam = lame;
                        if(inHalfspace(p,z3, q3)){
                                *statePtr = 1;
                                *done = true;
                        }

                        break;
                default: break;


        }


}
