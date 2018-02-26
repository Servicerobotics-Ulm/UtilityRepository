//--------------------------------------------------------------------------
//
//  Copyright (C) 2015 Timo Blender
//
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

#include <stdio.h>
#include "smartCdlCons.h"
#include "boost/multi_array.hpp"

class FileFunctions {

public:

	static int saveDist(std::string filename, boost::multi_array<short, 4> &distanceValues, boost::multi_array<short, 4> &distanceValuesAngular, int nrICCIndices);
	static int loadDist(std::string filename, boost::multi_array<short, 4> &distanceValues, boost::multi_array<short, 4> &distanceValuesAngular, int nrICCIndices);
	static int saveMaxAccClassic(std::string filename, double *cdl_curvature_vacc, double *cdl_curvature_wacc);
	static int saveCurvatureIndexRadius(std::string filename, int cdl_index_radius[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS]);
	static int loadCurvatureIndexRadius(std::string filename, int cdl_index_radius[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS]);
	static int saveCurvatureIndexICC(std::string filename, int cdl_index_icc[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS]);
	static int loadCurvatureIndexICC(std::string filename, int cdl_index_icc[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS]);
	static int saveOnlyAngleDist(std::string filename, short cdl_alph_lookup[CDL_MAX_X_CELLS][CDL_MAX_Y_CELLS][CURVATURE_INDICES_RADIUS][CURVATURE_INDICES_ICC_COARSE]);
	static int saveRadiusCurvature(std::string filename, double radii[CURVATURE_INDICES_RADIUS]);
	static int loadRadiusCurvature(std::string filename, double radii[CURVATURE_INDICES_RADIUS]);
	static int saveRadiusTriple(std::string filename, double radiusOfTriple[CURVATURE_INDICES_RADIUS]);
	static int loadRadiusTriple(std::string filename, double radiusOfTriple[CURVATURE_INDICES_RADIUS]);
	static int saveICCLocationX(std::string filename, boost::multi_array<double, 2> &iccLocationX, int nrICCIndices);
	static int loadICCLocationX(std::string filename, boost::multi_array<double, 2> &iccLocationX, int nrICCIndices);
	static int saveICCLocationY(std::string filename, boost::multi_array<double, 2> &iccLocationY, int nrICCIndices);
	static int loadICCLocationY(std::string filename, boost::multi_array<double, 2> &iccLocationY, int nrICCIndices);
	static int saveICCLocationXT(std::string filename, boost::multi_array<double, 2> &iccLocationXT, int nrICCIndices);
	static int loadICCLocationXT(std::string filename, boost::multi_array<double, 2> &iccLocationXT, int nrICCIndices);
	static int saveICCLocationYT(std::string filename, boost::multi_array<double, 2> &iccLocationYT, int nrICCIndices);
	static int loadICCLocationYT(std::string filename, boost::multi_array<double, 2> &iccLocationYT, int nrICCIndices);
};

