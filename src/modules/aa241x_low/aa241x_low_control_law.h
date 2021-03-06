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
 * @file aa241x_low.h
 *
 * Header file for student's low priority control law.
 * Runs at ~ XX HZ TODO: figure out XX
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */
#ifndef AA241X_LOW_H_
#define AA241X_LOW_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

/*
 * Declare variables here that you may want to access
 * in multiple different function.
 */

float PI = (float)M_PI;

/*
 * Declare function prototypes here.
 */

struct twoDvec{
        float x;
        float y;
};

twoDvec makeTwoDvec(float, float);

struct Matrix{
        float x11;
        float x12;
        float x21;
        float x22;
};

struct dubinsParams{
        float L;
        twoDvec cs;
        int lams;
        twoDvec ce;
        int lame;
        twoDvec z1;
        twoDvec q1;
        twoDvec z2;
        twoDvec z3;
        twoDvec q3;
};

struct waypointParams{
        int flag;
        twoDvec r;
        twoDvec q;
        twoDvec c;
        float rho;
        int lam;
};

struct waypoint{
        twoDvec xy;
        float heading;
        float radius;
};

twoDvec mulMxVec2D(Matrix, twoDvec);

twoDvec addVecs2D(twoDvec, twoDvec);

twoDvec scale2Dvec(float, twoDvec);

twoDvec subVecs2D(twoDvec, twoDvec);

float norm2D(twoDvec);

float dot2D(twoDvec, twoDvec);

float brk(float);

Matrix R_z(float);

bool circleOutOfBounds( twoDvec, twoDvec, twoDvec, float );

bool circleOutOfAnyBounds(twoDvec, float);

void findDubinsParams( waypoint, waypoint, float, dubinsParams*);

bool inHalfspace(twoDvec, twoDvec, twoDvec);

void followWaypointsDubins(dubinsParams, twoDvec, float, int*, bool*, waypointParams*);

void applyBestHeadings(waypoint [], int);

void shortestDubinsPath(waypoint [], float, int, float);

bool reachedWaypoint(twoDvec, waypoint);




#endif /* AA241X_SLOW_H_ */
