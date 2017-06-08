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
#include <array>        // array::size
#include <algorithm>    // std::next_permutation, std::sort

#include <uORB/uORB.h>

using namespace aa241x_low;

//const int num_waypoints = 4;
//float waypoint_Ns [num_waypoints] = {-2550.0f, -2320.0f, -2500.0f, -2300.0f };
//float waypoint_Es [num_waypoints] = {1840.0f, 1840.0f, 1900.0f, 1900.0f};
//float waypoint_Bs [num_waypoints] = {10.0f,10.0f,10.0f,10.0f};

int P_ind = 1;
int follow_state = 1;
bool follow_done = false;
bool path_done = false;
bool mission_done = false;


float mission_E [3][5] = {{-82.7353f,-39.0750f, -36.0992f,-1.0f,-1.0f},
                          {-44.1606f, -102.6630f, -97.5774f, -17.8668f, -107.7486f},
                          {-29.3185f, -81.0401f, -20.8425f, -34.4041f, -1.0f}};
float mission_N [3][5] = {{114.8850f, -162.4079f, 38.5720f, -1.0f, -1.0f},
                          {-102.6238f, 113.1855f, 53.4014f, 60.1995f, 172.9696f},
                          {-41.1402f, 94.9570f, -140.7803f, 18.6439f, -1.0f}};

float mission_R [3][5] = {{1.0f,1.0f,1.0f,-1.0f,-1.0f},
                          {1.0f,1.0f,1.0f,1.0f,1.0f},
                          {1.0f,1.0f,1.0f,1.0f,-1.0f}};

twoDvec q_boundaries[4] = { makeTwoDvec(-0.9964f, 0.0850f), makeTwoDvec(-0.0848f, -0.9964f), 
    makeTwoDvec(0.9964f,   -0.0850f), makeTwoDvec(0.0848f,    0.9964f) };
twoDvec z_boundaries[4] = { makeTwoDvec( -203.4000f,  -42.9000f), makeTwoDvec(0.7500f, -120.5000f),
    makeTwoDvec(215.1000f,  -78.5000f), makeTwoDvec(10.9500f,   -0.9000f) };

int mission_ctr = 0;

waypoint* P;
int num_waypoints = 0;


uint64_t miss_start_time;

int prev_phase_num = -100;
bool mission_phase_change = false;


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
    // float init_time = high_data.field15;
    // float elapsed_time_s = (hrt_absolute_time() - init_time)/1000000.0f;

    float R = aal_parameters.turn_radius;

    mission_phase_change = (phase_num != prev_phase_num);
    prev_phase_num = phase_num;

    bool reset = hrt_absolute_time() - previous_loop_timestamp > 1000000.0f;
    mission_done = !( (phase_num==1) || (phase_num==2) || (phase_num==3) ) || mission_failed;

    if (reset || mission_phase_change) { // Run if more than 1.0 seconds have passes since last loop,
        
        // set a heading hold for until computation finishes
        low_data.field1 = position_N;
        low_data.field2 = position_E;
        low_data.field3 = cosf(yaw);
        low_data.field4 = sinf(yaw);
        low_data.field5 = 1;

        // init mission
        num_waypoints = 0;
        for(int i=0;i<5;i++){
            // if(mission_R[mission_ctr][i] > 0.0f) num_waypoints++;
            if(plume_radius[i] > 0.0f) num_waypoints++;
            else break;
        }
        P = new waypoint[num_waypoints+2];


        // set up waypoints
        P[0].xy = makeTwoDvec(position_N, position_E);
        for(int i=0; i<num_waypoints; i++){
            //P[i+1].xy = makeTwoDvec(mission_N[mission_ctr][i],mission_E[mission_ctr][i]);
            P[i+1].xy = makeTwoDvec(plume_N[i],plume_E[i]);
        }

        P[num_waypoints+1].xy = makeTwoDvec(10.0f, -60.0f); // loiter location 

        P[0].heading = yaw;
        applyBestHeadings(P,num_waypoints+2);

        // reorder waypoints for shortest path length
        float u_command = high_data.field5; // velocity
        shortestDubinsPath(P,R,num_waypoints+2, 0.8f*u_command);


        P_ind = 1;
        follow_state = 1;
        follow_done = false;
        path_done = false;
        mission_done = false;
    }

    dubinsParams dbParams;
    waypointParams wpParams;
    if(!mission_done && !path_done && !follow_done){
        
        findDubinsParams(P[P_ind-1],P[P_ind],R,&dbParams);
        twoDvec pos = makeTwoDvec(position_N, position_E);
        followWaypointsDubins(dbParams, pos, R, &follow_state, &follow_done, &wpParams);
    }

    if(follow_done){
        P_ind++;
        if(P_ind > num_waypoints+1){
            path_done = true;
        }
        else {
            P[P_ind-1].xy = makeTwoDvec(position_N, position_E);
            P[P_ind-1].heading = yaw;
            follow_done = false;
        }
    }

    if(path_done || mission_done){
        // loiter
        low_data.field1 = P[num_waypoints+1].xy.x;
        low_data.field2 = P[num_waypoints+1].xy.y;
        low_data.field3 = R;
        low_data.field4 = 1;
        low_data.field5 = 2;
    }

    else if(wpParams.flag == 2){ // orbit
        low_data.field1 = wpParams.c.x;
        low_data.field2 = wpParams.c.y;
        low_data.field3 = R;
        low_data.field4 = wpParams.lam;
        low_data.field5 = wpParams.flag;

    }
    else{ // line follow
        low_data.field1 = wpParams.r.x;
        low_data.field2 = wpParams.r.y;
        low_data.field3 = wpParams.q.x;
        low_data.field4 = wpParams.q.y;
        low_data.field5 = wpParams.flag;
    }

    low_data.field6 = phase_num;
    low_data.field7 = num_waypoints;

    low_data.field8 = P_ind;
    low_data.field9 = P[P_ind].xy.x;
    low_data.field10 = P[P_ind].xy.y;
    low_data.field11 = follow_state;

    low_data.field12 = follow_done;
    low_data.field13 = path_done;
    low_data.field14 = mission_done;
    low_data.field15 = aal_parameters.boundary_margin;


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

float dot2D(twoDvec v1, twoDvec v2){
    return v1.x * v2.x + v1.y * v2.y;
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

bool circleOutOfBounds( twoDvec c, twoDvec z, twoDvec q, float R ){
    return ( dot2D(subVecs2D(c,z),q) + R + aal_parameters.boundary_margin > 0.0f );
}

bool circleOutOfAnyBounds(twoDvec c, float R){
    bool oob = false;
    for(int i = 0; i<4; i++){
        oob = oob || circleOutOfBounds(c,z_boundaries[i],q_boundaries[i], R);
    }
    return oob;

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

    bool crs_oob = circleOutOfAnyBounds(crs,R);
    bool cls_oob = circleOutOfAnyBounds(cls,R);
    bool cre_oob = circleOutOfAnyBounds(cre,R);
    bool cle_oob = circleOutOfAnyBounds(cle,R);

    // Compute L1
    twoDvec line = subVecs2D(cre, crs);
    float v = atan2f(line.y, line.x);
    float L1 = norm2D(line) + 
        R * brk(2.0f*PI - brk(v - PI/2.0f) - brk(psis - PI/2.0f)) + 
        R * brk(2.0f*PI + brk(psie - PI/2.0f) - brk(v - PI/2.0f));

    if(cre_oob || crs_oob){
        L1 = 1000000.0f;
    }


    // Compute L2
    line = subVecs2D(cle, crs);
    v = atan2f(line.y, line.x);
    float l = norm2D(line);
    float L2;
    float v2;
    if(2.0f*R < l){
        v2 = v - PI/2.0f + asinf(2.0f*R/l);
        L2 = sqrtf(powf(l,2) - 4.0f*powf(R,2)) +
                R * brk(2.0f*PI + brk(v2) - brk(psis - PI/2.0f)) +
                R * brk(2.0f*PI + brk(v2 + PI) - brk(psie + PI/2.0f));
    }
    else{
        L2 = 1000000.0f;
    }

    if(cle_oob || crs_oob){
        L2 = 1000000.0f;
    }


    // Compute L3
    line = subVecs2D(cre, cls);
    v = atan2f(line.y, line.x);
    l = norm2D(line);
    float L3;
    if(2.0f*R < l){
        v2 = acosf(2.0f*R/l);
        L3 = sqrtf(powf(l,2) - 4.0f*powf(R,2)) +
                R * brk(2.0f*PI + brk(psis + PI/2.0f) - brk(v+v2)) +
                R * brk(2.0f*PI + brk(psie - PI/2.0f) - brk(v+v2 -  PI));
    }
    else{
        L3 = 1000000.0f;
    }


    if(cre_oob || cls_oob){
        L3 = 1000000.0f;
    }


    // Compute L4
    line = subVecs2D(cle, cls);
    v = atan2f(line.y, line.x);
    l = norm2D(line);
    float L4 = l + 
        R * brk(2.0f*PI + brk(psis + PI/2.0f) - brk(v + PI/2.0f)) + 
        R * brk(2.0f*PI + brk(v + PI/2.0f) - brk(psie + PI/2.0f));

    if(cle_oob || cls_oob ){
        L4 = 1000000.f;
    }


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

void applyBestHeadings(waypoint P_[], int n){
    // n is len(P)
    twoDvec diff;
    for(int i=1; i<n-1; i++){
        diff = subVecs2D( P_[i+1].xy , P_[i-1].xy );
        P_[i].heading = atan2f(diff.y,diff.x);
    }
    diff = subVecs2D(P_[n-1].xy, P_[n-2].xy);
    P_[n-1].heading = atan2f(diff.y,diff.x);
}


void shortestDubinsPath(waypoint P_[], float R, int n, float vel) {
    // R is turning radius
    // n is length(P_array)
  
  int indicies[n] = {};
  for (int j = 0; j < n; j++) {
    indicies[j] = j;  }

  waypoint P_tmp[n];
  waypoint best_P[n];
  // as a failsafe copy the original path in
  for(int i = 0; i<n; i++){
    best_P[i] = P_[i];
  }

  float best_P_len = 100000.0f;
  int best_P_num_waypoints_hit = 0;

  float total_length = 0.0f;
  int num_waypoints_hit = 0;
  dubinsParams dbParams;

  do {

    total_length = 0.0f;
    for (int i = 0; i<n; i++){
        // create permutation of the waypoints
        P_tmp[i] = P_[indicies[i]];
    }
    applyBestHeadings(P_tmp, n);
    // compute length for each pair and add to total
    for (int i = 1; i<n; i++){
        findDubinsParams(P_tmp[i-1], P_tmp[i], R, &dbParams);
        total_length += dbParams.L;

        if(total_length / vel < 30.0f){
            num_waypoints_hit++;
        }
    }

    // cout << "total_len: " << total_length << "\n";

    if(num_waypoints_hit >= best_P_num_waypoints_hit){
        if((total_length < best_P_len) || (num_waypoints_hit > best_P_num_waypoints_hit) ){
            best_P_num_waypoints_hit = num_waypoints_hit;
            best_P_len = total_length;
            for(int i = 0; i<n; i++){
                best_P[i] = P_tmp[i];
            }
        }
    }

  } while ( std::next_permutation(indicies+1,indicies+n-1) );

  for(int i = 0; i<n; i++){
    P_[i] = best_P[i];
  }

}


