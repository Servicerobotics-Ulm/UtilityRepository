//--------------------------------------------------------------------------
//
//  Copyright (C) 1997/2008/2014 Christian Schlegel, Andreas Steck, Matthias Lutz
// 				  2015 Timo Blender
//
//        schlegel@hs-ulm.de
//        blender@hs-ulm.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
//
//  This file is part of the "SmartSoft OmniCDL component".
//  It provides navigation services based on the extension of the
//  Curvature Distance Lookup approach for omni-drives (3-DOFs).
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// --------------------------------------------------------------------------

#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include "smartCdlCons.h"
#include "smartCdlTypes.h"
#include "GeometryFunctions.h"
#include "FileFunctions.h"

using namespace std;

int ACCURACY_REDUCTION_FACTOR;
int stepSize, oneSideLength, nrICCIndices;
std::vector<double> curvatureIndicesICC;
boost::multi_array<short, 4> distanceValues;
boost::multi_array<short, 4> distanceValuesAngular;

double rad(double a) {
	return (a / 180.0 * M_PI);
}

double deg(double a) {
	return (a * 180.0 / M_PI);
}

double EPS = 2.22044604925031e-16;

int curvatureIndicesRadiusCounter = 0;
double curvatureIndicesRadius[CURVATURE_INDICES_RADIUS];
double radii[CURVATURE_INDICES_RADIUS];

std::vector<double> iccsA;
std::vector<double> iccsAL;

int curvatureIndicesIccCounter = 0;

int cdl_index_radius[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS];
int cdl_index_icc[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS];

struct cdl_index_struct cdl_index_data[CURVATURE_INDICES_RADIUS][CURVATURE_INDICES_ICC_COARSE];

struct cdl_polygon_struct polygon;

double cdl_curvature_vacc[CURVATURE_INDICES_RADIUS];
double cdl_curvature_wacc[CURVATURE_INDICES_RADIUS];

double radiusOfTriple[CURVATURE_INDICES_RADIUS];

boost::multi_array<double, 2> iccLocationX;
boost::multi_array<double, 2> iccLocationY;
boost::multi_array<double, 2> iccLocationXT;
boost::multi_array<double, 2> iccLocationYT;

void calculateMaximumAccelerations(double v, double w, int curvatureCounter) {

	double cdl_v_rot_step = rad(CDL_V_ROT_STEP);
	double cdl_a_rot_max  = rad(CDL_A_ROT_MAX);
	double cdl_a_tra_max  = CDL_A_TRA_MAX;

	// Calculate maximum accelerations for current curvature index
	if (fabs(v) < CDL_V_TRA_STEP/2.0) {

		// No translation => rotate at the current position
		cdl_curvature_vacc[curvatureCounter] = 0.0;
		cdl_curvature_wacc[curvatureCounter] = cdl_a_rot_max;
	}
	else if (fabs(w) < cdl_v_rot_step/2.0) {
		// No rotation => driving straight ahead
		cdl_curvature_vacc[curvatureCounter] = cdl_a_tra_max;
		cdl_curvature_wacc[curvatureCounter] = 0.0;
	}
	else {
		// Translation and rotation
		cdl_curvature_vacc[curvatureCounter] = min(cdl_a_tra_max, fabs(cdl_a_rot_max*v/w));
		cdl_curvature_wacc[curvatureCounter] = min(cdl_a_rot_max, fabs(cdl_a_tra_max*w/v));
	}
}

void determineCurvatureIndicesRadiusPrecise() {

	double v = 0.0;
	double w = 0.0;
	int vi = 0;
	int wi = 0;

	double cdl_v_rot_min = rad(CDL_V_ROT_MIN);
	double cdl_v_rot_step = rad(CDL_V_ROT_STEP);

	curvatureIndicesRadiusCounter = 0;

	// Outer cells of upper half of the discretization window is sufficient
	for (vi = 141; vi < CDL_TRA_CELLS - 1; vi++) {
		v = CDL_V_TRA_MIN + (vi * CDL_V_TRA_STEP);
		w = cdl_v_rot_min + (wi * cdl_v_rot_step);
		double alpha = atan2(v,w);
		curvatureIndicesRadius[curvatureIndicesRadiusCounter] = alpha;
		radii[curvatureIndicesRadiusCounter] = v / w;
		calculateMaximumAccelerations(v,w,curvatureIndicesRadiusCounter);
		curvatureIndicesRadiusCounter++;
	}

	vi = CDL_TRA_CELLS - 1;

	for (wi = 0; wi < CDL_ROT_CELLS - 1; wi++) {
		v = CDL_V_TRA_MIN + (vi * CDL_V_TRA_STEP);
		w = cdl_v_rot_min + (wi * cdl_v_rot_step);
		double alpha = atan2(v,w);
		curvatureIndicesRadius[curvatureIndicesRadiusCounter] = alpha;
		if (wi == 70) {
			radii[curvatureIndicesRadiusCounter] = 0.0;
		}
		else {
			radii[curvatureIndicesRadiusCounter] = v / w;
		}
		calculateMaximumAccelerations(v,w,curvatureIndicesRadiusCounter);
		curvatureIndicesRadiusCounter++;
	}

	wi = CDL_ROT_CELLS - 1;

	for (vi = CDL_TRA_CELLS - 1; vi > 140; vi--) {
		v = CDL_V_TRA_MIN + (vi * CDL_V_TRA_STEP);
		w = cdl_v_rot_min + (wi * cdl_v_rot_step);
		double alpha = atan2(v,w);
		curvatureIndicesRadius[curvatureIndicesRadiusCounter] = alpha;
		radii[curvatureIndicesRadiusCounter] = v / w;
		calculateMaximumAccelerations(v,w,curvatureIndicesRadiusCounter);
		curvatureIndicesRadiusCounter++;
	}

	cout << "Curvature indices for radius: " << curvatureIndicesRadiusCounter << endl;
}

void determineCurvatureIndicesICC() {

	double vx = 0.0;
	double vy = 0.0;
	int vxi = 0;
	int vyi = 0;

	curvatureIndicesIccCounter = 0;

	for (vxi = 0; vxi < oneSideLength - 1; vxi++) {
		vx = CDL_V_TRA_MIN_STANDARD + (vxi * stepSize);
		vy = CDL_V_TRA_MIN_STANDARD + (vyi * stepSize);
		curvatureIndicesICC[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsA[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsAL[curvatureIndicesIccCounter] = atan2(vx,vy);
		curvatureIndicesIccCounter++;
	}


	vxi = oneSideLength - 1;

	for (vyi = 0; vyi < oneSideLength - 1; vyi++) {
		vx = CDL_V_TRA_MIN_STANDARD + (vxi * stepSize);
		vy = CDL_V_TRA_MIN_STANDARD + (vyi * stepSize);
		curvatureIndicesICC[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsA[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsAL[curvatureIndicesIccCounter] = atan2(vx,vy);
		curvatureIndicesIccCounter++;
	}

	vyi = oneSideLength - 1;

	for (vxi = oneSideLength - 1; vxi > 0; vxi--) {
		vx = CDL_V_TRA_MIN_STANDARD + (vxi * stepSize);
		vy = CDL_V_TRA_MIN_STANDARD + (vyi * stepSize);
		curvatureIndicesICC[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsA[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsAL[curvatureIndicesIccCounter] = atan2(vx,vy);
		curvatureIndicesIccCounter++;
	}

	vxi = 0;

	for (vyi = oneSideLength - 1; vyi > 0; vyi--) {
		vx = CDL_V_TRA_MIN_STANDARD + (vxi * stepSize);
		vy = CDL_V_TRA_MIN_STANDARD + (vyi * stepSize);
		curvatureIndicesICC[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsA[curvatureIndicesIccCounter] = atan2(vy,vx);
		iccsAL[curvatureIndicesIccCounter] = atan2(vx,vy);
		curvatureIndicesIccCounter++;
	}

	cout << "Curvature indices for ICC-line: " << curvatureIndicesIccCounter << endl;
}

void assignCurvatureIndicesToVelocityTripels() {

	double vx = 0.0;
	double vy = 0.0;
	double w = 0.0;
	double vGes = 0.0;
	double alpha = 0.0;
	double beta = 0.0;
	double radius = 0.0;
	double diffOld = 0.0;
	double diff = 0.0;
	int indexRadius = 0;
	int indexICC = 0;

	double iccX = 0.0;
	double iccY = 0.0;
	double iccXT = 0.0;
	double iccYT = 0.0;

	double cdl_v_rot_min = rad(CDL_V_ROT_MIN);
	double cdl_v_rot_step = rad(CDL_V_ROT_STEP);

	for (int vxi = 0; vxi < CDL_TRA_CELLS_CLASSIC; vxi++) {
		for (int vyi = 0; vyi < CDL_TRA_CELLS_CLASSIC; vyi++) {
			for (int wi = 0; wi < CDL_ROT_CELLS; wi++) {

				vx = CDL_V_TRA_MIN_STANDARD + (vxi * CDL_V_TRA_STEP);
				vy = CDL_V_TRA_MIN_STANDARD + (vyi * CDL_V_TRA_STEP);
				w = cdl_v_rot_min + (wi * cdl_v_rot_step);

				vGes = sqrt(pow(vx, 2) + pow(vy, 2));
				alpha = atan2(vGes, w);
				beta = atan2(vy, vx);

				if (wi == 70) {
					radius = 0.0;
				}
				else {
					radius = fabs(vGes / w);
				}

				diffOld = 1000.0;
				for (int i = 0; i < curvatureIndicesRadiusCounter; i++) {
					diff = fabs(alpha - curvatureIndicesRadius[i]);
					if (diff < diffOld) {
						cdl_index_radius[vxi][vyi][wi] = i;
						indexRadius = i;
						diffOld = diff;
					}
				}

				diffOld = 1000.0;
				for (int i = 0; i < curvatureIndicesIccCounter; i++) {
					diff = fabs(beta - curvatureIndicesICC[i]);
					if (diff < diffOld) {
						cdl_index_icc[vxi][vyi][wi] = i;
						indexICC = i;
						diffOld = diff;
					}
				}

				if (vxi == 100 && vyi == 100 && wi == 70) {
					// No motion
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi == 100 && vyi == 100 && wi < 70) {
					// Turn on place clockwise
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi == 100 && vyi == 100 && wi > 70) {
					// Turn on place counterclockwise
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi < 100 && vyi == 100 && wi < 70) {
					// Normal curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi > 100 && vyi == 100 && wi < 70) {
					// Normal curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi < 100 && vyi == 100 && wi > 70) {
					// Normal curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi > 100 && vyi == 100 && wi > 70) {
					// Normal curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi == 100 && vyi < 100 && wi < 70) {
					// Sideway curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi == 100 && vyi > 100 && wi < 70) {
					// Sideway curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi == 100 && vyi < 100 && wi > 70) {
					// Sideway curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi == 100 && vyi > 100 && wi > 70) {
					// Sideway curve
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi < 100 && vyi == 100 && wi == 70) {
					// Straight backward
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi > 100 && vyi == 100 && wi == 70) {
					// Straight forward
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 0;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi == 100 && vyi < 100 && wi == 70) {
					// Straight right
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi == 100 && vyi > 100 && wi == 70) {
					// Straight left
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 0;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi < 100 && vyi < 100 && wi == 70) {
					// Linear fourth quadrant
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi < 100 && vyi > 100 && wi == 70) {
					// Linear third quadrant
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi > 100 && vyi > 100 && wi == 70) {
					// Linear second quadrant
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				else if (vxi > 100 && vyi < 100 && wi == 70) {
					// Linear first quadrant
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 0;
				}

				// else special curves
				else if (vxi < 100 && vyi < 100 && wi < 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi > 100 && vyi < 100 && wi < 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi < 100 && vyi > 100 && wi < 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi > 100 && vyi > 100 && wi < 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = -1;
				}

				else if (vxi < 100 && vyi < 100 && wi > 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi > 100 && vyi < 100 && wi > 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = -1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi < 100 && vyi > 100 && wi > 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = -1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else if (vxi > 100 && vyi > 100 && wi > 70) {
					cdl_index_data[indexRadius][indexICC].trans_dir_x = 1;
					cdl_index_data[indexRadius][indexICC].trans_dir_y = 1;
					cdl_index_data[indexRadius][indexICC].rot_dir = 1;
				}

				else {
					cout << "Error !" << endl;
					cout << vx << "/" << vy << "/" << w << endl;
				}

				if (wi == 70) {
					iccX = 0.0;
					iccY = 0.0;
					iccXT = 0.0;
					iccYT = 0.0;
				}

				else {

					// Sign must be set manually due to the used coordinate system

					// (0 and left and cc) or (0 and right and c)
					if ((vx < EPS && vy > 0 && w > 0)
							|| (vx < EPS && vy < 0 && w < 0)) {
						// ICC location = negative x-axis
						iccX = -1 * fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = -1 * fabs(radius) * fabs(sin(beta));
						iccYT = fabs(radius) * fabs(cos(beta));
					}

					// (forward and left and cc) or (backward and right and c)
					if ((vx > 0 && vy > 0 && w > 0)
							|| (vx < 0 && vy < 0 && w < 0)) {
						// ICC location = between positive y-axis and negative x-axis (3. quadrant)
						iccX = -1 * fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = -1 * fabs(radius) * fabs(sin(beta));
						iccYT = fabs(radius) * fabs(cos(beta));
					}

					// (forward and 0 and c) or (backward and 0 and cc)
					if ((vx > 0 && vy < EPS && w < 0)
							|| (vx < 0 && vy < EPS && w > 0)) {
						// ICC location = negative y-axis
						iccX = -1 * fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = -1 * fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = -1 * fabs(radius) * fabs(sin(beta));
						iccYT = -1 * fabs(radius) * fabs(cos(beta));
					}

					// (backward and left and cc) or (forward and right and c)
					if ((vx < 0 && vy > 0 && w > 0)
							|| (vx > 0 && vy < 0 && w < 0)) {
						// ICC location = between negative x-axis and negative y-axis (4. quadrant)
						iccX = -1 * fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = -1 * fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = -1 * fabs(radius) * fabs(sin(beta));
						iccYT = -1 * fabs(radius) * fabs(cos(beta));
					}

					// (0 and left and c) or (0 and right and cc)
					if ((vx < EPS && vy > 0 && w < 0)
							|| (vx < EPS && vy < 0 && w > 0)) {
						// ICC location = positive x-axis
						iccX = fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = -1 * fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = fabs(radius) * fabs(sin(beta));
						iccYT = -1 * fabs(radius) * fabs(cos(beta));
					}

					// (backward and right and cc) or (forward and left and c)
					if ((vx < 0 && vy < 0 && w > 0)
							|| (vx > 0 && vy > 0 && w < 0)) {
						// ICC location = between negative y-axis and positive x-axis (1. quadrant)
						iccX = fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = -1 * fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = fabs(radius) * fabs(sin(beta));
						iccYT = -1 * fabs(radius) * fabs(cos(beta));
					}

					// (forward and 0 and cc) or (backward and 0 and c)
					if ((vx > 0 && vy < EPS && w > 0)
							|| (vx < 0 && vy < EPS && w < 0)) {
						// ICC location = positive y-axis
						iccX = fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = fabs(radius) * fabs(sin(beta));
						iccYT = fabs(radius) * fabs(cos(beta));
					}

					// (forward and right and cc) or (backward and left and c)
					if ((vx > 0 && vy < 0 && w > 0)
							|| (vx < 0 && vy > 0 && w < 0)) {
						// ICC location = between positive x-axis and positive y-axis (2. quadrant)
						iccX = fabs(radii[indexRadius])
								* fabs(sin(iccsA[indexICC]));
						iccY = fabs(radii[indexRadius])
								* fabs(cos(iccsA[indexICC]));
						iccXT = fabs(radius) * fabs(sin(beta));
						iccYT = fabs(radius) * fabs(cos(beta));
					}
				}
				cdl_index_data[indexRadius][indexICC].iccX = iccX;
				cdl_index_data[indexRadius][indexICC].iccY = iccY;
				cdl_index_data[indexRadius][indexICC].radius = radii[indexRadius];

				if (wi == 70) {
					cdl_index_data[indexRadius][indexICC].slope = iccsAL[indexICC];
				}
				else {
					cdl_index_data[indexRadius][indexICC].slope = iccsA[indexICC];
				}

				radiusOfTriple[indexRadius] = radius;
				iccLocationX[indexRadius][indexICC] = iccX;
				iccLocationY[indexRadius][indexICC] = iccY;
				iccLocationXT[indexRadius][indexICC] = iccXT;
				iccLocationYT[indexRadius][indexICC] = iccYT;
			}
		}
	}
}

int calculateDistance(struct cdl_polygon_struct *polygon) {

	int xi, yi, seg;
	double x, y;
	int status;

	int trans_x, trans_y;
	int rot;
	double distance;
	double angle;

	double min_distance;
	double min_angle;

	double slope;

	double iccX, iccY;

	for (xi = 0; xi < CDL_C_X_CELLS; xi++) {         // x-index cartesian space
		x = CDL_C_X_MIN + xi * CDL_C_RES;
		for (yi = 0; yi < CDL_C_Y_CELLS; yi++) {      // y-index cartesian space
			y = CDL_C_Y_MIN + yi * CDL_C_RES;
			status = GeometryFunctions::cdl_check_point_inside_polygon(x, y, polygon);
			if (status == CDL_OK) {

				// Point p is inside robot => remaining distance zero
				for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
					for (int j = 0; j < nrICCIndices; j++) {
						distanceValues[xi][yi][i][j] = 0;
						distanceValuesAngular[xi][yi][i][j] = 0;
					}
				}
			}

			else {

				// Point p is outside of robot => calculate distance

				for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {

					for (int j = 0; j < nrICCIndices; j++) {

						min_distance = CDL_MAX_DISTANCE;
						min_angle = 2 * M_PI;

						for (seg = 0; seg < polygon->number_of_segments; seg++) {

							trans_x = cdl_index_data[i][j].trans_dir_x;
							trans_y = cdl_index_data[i][j].trans_dir_y;
							rot = cdl_index_data[i][j].rot_dir;
							slope = cdl_index_data[i][j].slope;
							iccX = cdl_index_data[i][j].iccX;
							iccY = cdl_index_data[i][j].iccY;

							if (trans_x == 0 && trans_y == 0 && rot == 0) {
								// Robot does not move
								distance = 0.0;
								angle = 0.0;
							}

							// Linear motions
							else if (rot == 0) {

								if (trans_x != 0 && trans_y == 0) {
									// Straight forward or backward
									GeometryFunctions::cdl_calc_distance_straight(
											x, y, polygon->line[seg].x1,
											polygon->line[seg].y1,
											polygon->line[seg].x2,
											polygon->line[seg].y2, trans_x,
											&distance);
								}

								else if (trans_x == 0 && trans_y != 0) {
									// Straight left or right
									GeometryFunctions::cdl_calc_distance_straight_sideway(
											x, y, polygon->line[seg].x1,
											polygon->line[seg].y1,
											polygon->line[seg].x2,
											polygon->line[seg].y2, trans_y,
											&distance);
								}

								else {
									// Move a line not parallel to x-axis and not parallel to y-axis
									GeometryFunctions::cdl_calc_distance_linear(
											x, y, slope, polygon->line[seg].x1,
											polygon->line[seg].y1,
											polygon->line[seg].x2,
											polygon->line[seg].y2, trans_x,
											trans_y, &distance);
								}

								angle = 0.0;
							}

							else {

								// Normal curves, sideway curves and special curves
								status = GeometryFunctions::cdl_calc_distance(x,
										y, iccX, iccY, polygon->line[seg].x1,
										polygon->line[seg].y1,
										polygon->line[seg].x2,
										polygon->line[seg].y2, trans_x, rot,
										&distance, &angle);

							}

							min_distance = min(min_distance, distance);
							min_angle = min(min_angle, angle);
						}

					// Now all segments have been considered. The minimum distance
					// drivable using the current curvature is in min_distance. This
					// is not the distance of the hull of the robot until collision,
					// but the distance of the powered wheel until collision.
					// The according rotation angle is in min_alpha. Both values
					// are always positive, [0,CDL_MAX_DISTANCE], [0, 2 PI]

					distanceValues[xi][yi][i][j] = (short) min_distance;
					distanceValuesAngular[xi][yi][i][j] = (short) (min_angle * 1000.0);

					}
				}
			}
		}
	}

	return CDL_OK;
}

int main() {

	std::string dir = "";

	int choice = 0;
	int status = 0;

	double vx = 0.0;
	double vy = 0.0;
	double w = 0.0;
	double x = 0.0;
	double y = 0.0;

	std::cout << "Calculate & write to file (1)" << std::endl;
	std::cout << "Load from file (2)" << std::endl;
	std::cout << "Quit (3)" << std::endl;
	std::cout << ">> ";cin >> choice;

	std::cout << "Enter ACCURACY_REDUCTION_FACTOR for ICC line indices:" << std::endl;
	std::cout << ">> ";cin >> ACCURACY_REDUCTION_FACTOR;

	stepSize = ACCURACY_REDUCTION_FACTOR*CDL_V_TRA_STEP;
	oneSideLength = (CDL_V_TRA_MAX_STANDARD-CDL_V_TRA_MIN_STANDARD)/stepSize+1;
	nrICCIndices = (oneSideLength-1)*4;

	curvatureIndicesICC.resize(nrICCIndices);
	iccsA.resize(nrICCIndices);
	iccsAL.resize(nrICCIndices);
	distanceValues.resize(boost::extents[CDL_MAX_X_CELLS][CDL_MAX_Y_CELLS][CURVATURE_INDICES_RADIUS][nrICCIndices]);
	distanceValuesAngular.resize(boost::extents[CDL_MAX_X_CELLS][CDL_MAX_Y_CELLS][CURVATURE_INDICES_RADIUS][nrICCIndices]);

	iccLocationX.resize(boost::extents[CURVATURE_INDICES_RADIUS][nrICCIndices]);
	iccLocationY.resize(boost::extents[CURVATURE_INDICES_RADIUS][nrICCIndices]);
	iccLocationXT.resize(boost::extents[CURVATURE_INDICES_RADIUS][nrICCIndices]);
	iccLocationYT.resize(boost::extents[CURVATURE_INDICES_RADIUS][nrICCIndices]);

	switch (choice) {
		case 1:

			// Robotino 3 contour
			polygon.number_of_segments = 8;
			polygon.line[0].x1 =  250.0;polygon.line[0].y1 = -124.0;  // AB
			polygon.line[0].x2 =  250.0;polygon.line[0].y2 =  124.0;

			polygon.line[1].x1 =  125.0;polygon.line[1].y1 = -249.0;  // BC
			polygon.line[1].x2 =  250.0;polygon.line[1].y2 = -124.0;

			polygon.line[2].x1 =   125.0;polygon.line[2].y1 = -249.0;  // CD
			polygon.line[2].x2 =  -125.0;polygon.line[2].y2 = -249.0;

			polygon.line[3].x1 = -125.0;polygon.line[3].y1 = -249.0;  // DE
			polygon.line[3].x2 = -250.0;polygon.line[3].y2 = -124.0;

			polygon.line[4].x1 =  -250.0;polygon.line[4].y1 = -124.0;  // EF
			polygon.line[4].x2 =  -250.0;polygon.line[4].y2 =  124.0;

			polygon.line[5].x1 =  -250.0;polygon.line[5].y1 = 124.0;  // FG
			polygon.line[5].x2 =  -125.0;polygon.line[5].y2 = 249.0;

			polygon.line[6].x1 =  -125.0;polygon.line[6].y1 =  249.0;  // GH
			polygon.line[6].x2 =  125.0;polygon.line[6].y2 = 249.0;

			polygon.line[7].x1 =  250.0;polygon.line[7].y1 = 124.0;  // HI
			polygon.line[7].x2 =  125.0;polygon.line[7].y2 = 249.0;

			determineCurvatureIndicesRadiusPrecise();

			determineCurvatureIndicesICC();

			assignCurvatureIndicesToVelocityTripels();

			std::cout << "Calculate distances ..." << std::endl;
			calculateDistance(&polygon);
			std::cout << "... finished." << std::endl;

			status = FileFunctions::saveDist(dir+"fullCoverageDistances.dat", distanceValues, distanceValuesAngular, nrICCIndices);
			if (status != CDL_OK) { cout << "Error saving distances." << endl; }

			status = FileFunctions::saveMaxAccClassic(dir+"CDLacc.dat", cdl_curvature_vacc, cdl_curvature_wacc);
			if (status!=CDL_OK) { cout << "Error saving max accelerations." << endl; }

			status = FileFunctions::saveCurvatureIndexRadius(dir+"CDLindex_radius.dat", cdl_index_radius);
			if (status!=CDL_OK) { cout << "Error saving curvature index radius." << endl; }

			status = FileFunctions::saveCurvatureIndexICC(dir+"CDLindex_icc.dat", cdl_index_icc);
			if (status!=CDL_OK) { cout << "Error saving curvature index icc." << endl; }

			status = FileFunctions::saveRadiusCurvature(dir+"radiusOfCurvature.dat", radii);
			if (status!=CDL_OK) { cout << "Error saving radius of curvature." << endl; }

			status = FileFunctions::saveRadiusTriple(dir+"radiusOfTriple.dat", radiusOfTriple);
			if (status!=CDL_OK) { cout << "Error saving radius of triple." << endl; }

			status = FileFunctions::saveICCLocationX(dir+"iccLocationX.dat", iccLocationX, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error saving iccLocationX." << endl; }
			status = FileFunctions::saveICCLocationY(dir+"iccLocationY.dat", iccLocationY, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error saving iccLocationY." << endl; }
			status = FileFunctions::saveICCLocationXT(dir+"iccLocationXT.dat", iccLocationXT, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error saving iccLocationXT." << endl; }
			status = FileFunctions::saveICCLocationYT(dir+"iccLocationYT.dat", iccLocationYT, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error saving iccLocationYT." << endl; }

			std::cout << "Lookup tables successfully written to files." << std::endl;

			break;

		case 2:

			status = FileFunctions::loadCurvatureIndexRadius(dir+"CDLindex_radius.dat", cdl_index_radius);
			if (status!=CDL_OK) { cout << "Error loading curvature indices radius." << endl; }

			status = FileFunctions::loadCurvatureIndexICC(dir+"CDLindex_icc.dat", cdl_index_icc);
			if (status!=CDL_OK) { cout << "Error loading curvature indices icc." << endl; }

			status = FileFunctions::loadDist(dir+"fullCoverageDistances.dat", distanceValues, distanceValuesAngular, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error loading distance values." << endl; }
			status = FileFunctions::loadRadiusCurvature(dir+"radiusOfCurvature.dat", radii);
			if (status!=CDL_OK) { cout << "Error loading radius of curvature." << endl; }
			status = FileFunctions::loadRadiusTriple(dir+"radiusOfTriple.dat", radiusOfTriple);
			if (status!=CDL_OK) { cout << "Error loading radius of triple." << endl; }

			status = FileFunctions::loadICCLocationX(dir+"iccLocationX.dat", iccLocationX, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error loading iccLocationX." << endl; }
			status = FileFunctions::loadICCLocationY(dir+"iccLocationY.dat", iccLocationY, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error loading iccLocationY." << endl; }
			status = FileFunctions::loadICCLocationXT(dir+"iccLocationXT.dat", iccLocationXT, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error loading iccLocationXT." << endl; }
			status = FileFunctions::loadICCLocationYT(dir+"iccLocationYT.dat", iccLocationYT, nrICCIndices);
			if (status!=CDL_OK) { cout << "Error loading iccLocationYT." << endl; }

			while(1) {

				printf("Ueberpruefung der Indizes in der curvature-Tabelle\n");
				printf("Eingabe vx           [mm/s]  ");fflush(stdout);scanf("%lf",&vx);
				printf("Eingabe vy           [mm/s]  ");fflush(stdout);scanf("%lf",&vy);
				printf("Eingabe w           [deg/s] ");fflush(stdout);scanf("%lf",&w);
				printf("Eingabe Hindernis x [mm]    ");fflush(stdout);scanf("%lf",&x);
				printf("Eingabe Hindernis y [mm]    ");fflush(stdout);scanf("%lf",&y);

				double cdl_v_rot_min = rad(CDL_V_ROT_MIN);
				double cdl_v_rot_step = rad(CDL_V_ROT_STEP);
				w = rad(w);

				int vxi = (int)(floor((vx-CDL_V_TRA_MIN_STANDARD)/CDL_V_TRA_STEP+0.5));
				int vyi = (int)(floor((vy-CDL_V_TRA_MIN_STANDARD)/CDL_V_TRA_STEP+0.5));
				int wi = (int)(floor((w-cdl_v_rot_min)/cdl_v_rot_step+0.5));
				int xi = (int)(ceil((x-CDL_C_X_MIN)/CDL_C_RES));
				int yi = (int)(ceil((y-CDL_C_Y_MIN)/CDL_C_RES));

				int ir = cdl_index_radius[vxi][vyi][wi];
				int il = cdl_index_icc[vxi][vyi][wi];
				int d = distanceValues[xi][yi][ir][il];
				double a = (distanceValuesAngular[xi][yi][ir][il]/1000.0*180.0/M_PI);
				double vacc = cdl_curvature_vacc[ir];
				double wacc = cdl_curvature_wacc[ir]*180.0/M_PI;

				printf("vx vy w %f %f %f  Index vxi, vyi, wi %d %d %d  CurvatureRadius CurvatureICC %d %d\n",vx,vy,w,vxi,vyi,wi,ir,il);
				printf("x y %f %f  Index xi,yi %d %d\n",x,y,xi,yi);
				printf("Distanz Winkel %d %f\n",d,a);
				printf("vacc wacc      %f %f\n",vacc,wacc);
				printf("Radius(Curvature Index), Radius(Velocity Triple)%f %f\n", radii[ir], radiusOfTriple[ir]);
				printf("ICCX ICCY (Curvature Index) %f %f\n", iccLocationX[ir][il], iccLocationY[ir][il]);
				printf("ICCX ICCY (Velocity Triple) %f %f\n", iccLocationXT[ir][il], iccLocationYT[ir][il]);
				printf("\n");
			}
			break;
		case 3:
			return 0;
		default:
			return 0;
	}

	return 0;
}
