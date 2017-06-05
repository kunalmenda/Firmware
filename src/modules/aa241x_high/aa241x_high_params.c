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
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"



/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_EXAMPLE_1, 10.0f);
PARAM_DEFINE_FLOAT(AAH_H_COMMAND, -80.0f); //Neg height above ground (m), adjust as needed
PARAM_DEFINE_FLOAT(AAH_U_COMMAND, 15.0f); //velocity
PARAM_DEFINE_FLOAT(AAH_PSI_COMMAND, 0.0f); //yaw
PARAM_DEFINE_FLOAT(AAH_K_U, 0.6f); // velocity (throttle)
PARAM_DEFINE_FLOAT(AAH_K_H, -0.2f); // altitude
PARAM_DEFINE_FLOAT(AAH_K_THETA, 4.0f); // elevator
PARAM_DEFINE_FLOAT(AAH_K_PHI, -2.0f); // aileron
PARAM_DEFINE_FLOAT(AAH_K_PSI, 1.0f);
PARAM_DEFINE_FLOAT(AAH_K_Y, 1.0f);
PARAM_DEFINE_FLOAT(AAH_K_CIRC, 1.0f);
PARAM_DEFINE_FLOAT(AAH_PSI_INF, 0.7f);
PARAM_DEFINE_FLOAT(AAH_THROT_TRIM, 0.6f);
/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_PROPROLLGAIN, 1.0f);

// TODO: define custom parameters here


int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	h->example_high_param		= param_find("AAH_EXAMPLE");
	h->proportional_roll_gain 	= param_find("AAH_PROPROLLGAIN");

	// TODO: add the above line for each of your custom parameters........
	h->h_command	= param_find("AAH_H_COMMAND");
	h->u_command	= param_find("AAH_U_COMMAND");
	h->psi_command	= param_find("AAH_PSI_COMMAND");

	h->k_u	= param_find("AAH_K_U");
	h->k_h	= param_find("AAH_K_H");
	h->k_theta	= param_find("AAH_K_THETA");
	h->k_phi	= param_find("AAH_K_PHI");
	h->k_psi	= param_find("AAH_K_PSI");	
	h->k_y	= param_find("AAH_K_Y");
	h->k_circ = param_find("AAH_K_CIRC");
	h->psi_inf = param_find("AAH_PSI_INF");
	h->throt_trim	= param_find("AAH_THROT_TRIM");
	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->example_high_param, &(p->example_high_param));
	param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));

	// TODO: add the above line for each of your custom parameters.....
	param_get(h->h_command, &(p->h_command));
	param_get(h->u_command, &(p->u_command));
	param_get(h->psi_command, &(p->psi_command));
	param_get(h->k_u, &(p->k_u));
	param_get(h->k_h, &(p->k_h));
	param_get(h->k_theta, &(p->k_theta));
	param_get(h->k_phi, &(p->k_phi));
	param_get(h->k_psi, &(p->k_psi));
	param_get(h->k_y, &(p->k_y));
	param_get(h->k_circ, &(p->k_circ));
	param_get(h->psi_inf, &(p->psi_inf));
	param_get(h->throt_trim, &(p->throt_trim));
	
	return OK;
}
