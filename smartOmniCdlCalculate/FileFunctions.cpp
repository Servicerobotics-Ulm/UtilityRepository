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

#include <iostream>
#include "FileFunctions.h"
int FileFunctions::saveDist(std::string filename, boost::multi_array<short, 4> &distanceValues, boost::multi_array<short, 4> &distanceValuesAngular, int nrICCIndices)
{
  FILE *file;
  int counter,size;
  double a;

  file = fopen(filename.c_str(),"wb");
  if (file!=NULL) {
    counter=0;
    size=0;

    a=(double)CDL_V_TRA_MAX_STANDARD;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_V_TRA_MIN_STANDARD;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_V_ROT_MAX;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_V_ROT_MIN;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_V_TRA_STEP;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_V_ROT_STEP;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_TRA_CELLS_CLASSIC;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_ROT_CELLS;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_X_MIN;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_X_MAX;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_Y_MIN;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_Y_MAX;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_RES;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_X_CELLS;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CDL_C_Y_CELLS;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)CURVATURE_INDICES_RADIUS;size+=fwrite(&a,sizeof(a),1,file);counter++;
    a=(double)nrICCIndices;size+=fwrite(&a,sizeof(a),1,file);counter++;

    if (size != counter) {
      fclose(file);
      return CDL_NOK;
    };

    size=fwrite(&(distanceValues[0][0][0][0]),
                sizeof(short),
                CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices,
                file);
    if (size != (CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices)) {
      fclose(file);
      return CDL_NOK;
    };

    size=fwrite(&(distanceValuesAngular[0][0][0][0]),
                sizeof(short),
                CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices,
                file);
    if (size != (CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices)) {
      fclose(file);
      return CDL_NOK;
    };

    fclose(file);
    return CDL_OK;
  }
  return CDL_NOK;
}

int FileFunctions::loadDist(std::string filename, boost::multi_array<short, 4> &distanceValues, boost::multi_array<short, 4> &distanceValuesAngular, int nrICCIndices)
{
  FILE *file;
  int counter,size;
  double a;

  file = fopen(filename.c_str(),"rb");
  if (file!=NULL) {
    counter=0;
    size=0;

    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_V_TRA_MAX_STANDARD) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_V_TRA_MIN_STANDARD) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_V_ROT_MAX) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_V_ROT_MIN) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_V_TRA_STEP) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_V_ROT_STEP) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_TRA_CELLS_CLASSIC) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_ROT_CELLS) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_X_MIN) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_X_MAX) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_Y_MIN) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_Y_MAX) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_RES) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_X_CELLS) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CDL_C_Y_CELLS) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)CURVATURE_INDICES_RADIUS) {
      fclose(file);
      return CDL_NOK;
    };
    size+=fread(&a,sizeof(a),1,file);counter++;
    if (a!=(double)nrICCIndices) {
      fclose(file);
      return CDL_NOK;
    };

    if (size != counter) {
      fclose(file);
      return CDL_NOK;
    };

    size=fread(&(distanceValues[0][0][0][0]),
                sizeof(short),
                CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices,
                file);
    if (size != (CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices)) {
      fclose(file);

      return CDL_NOK;
    };

    size=fread(&(distanceValuesAngular[0][0][0][0]),
                sizeof(short),
                CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices,
                file);
    if (size != (CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*nrICCIndices)) {
      fclose(file);
      return CDL_NOK;
    };

    fclose(file);
    return CDL_OK;
  }

  return CDL_NOK;
}

int FileFunctions::saveMaxAccClassic(std::string filename, double *cdl_curvature_vacc, double *cdl_curvature_wacc) {

	FILE *file;
	int size;
	int a;

	file = fopen(filename.c_str(), "wb");
	if (file != NULL) {
		a = (int) CURVATURE_INDICES_RADIUS;
		size = fwrite(&a, sizeof(a), 1, file);
		if (size != 1) {
			fclose(file);
			return CDL_NOK;
		}
		size = fwrite(&(cdl_curvature_vacc[0]), sizeof(double),
				CURVATURE_INDICES_RADIUS, file);
		if (size != CURVATURE_INDICES_RADIUS) {
			fclose(file);
			return CDL_NOK;
		}
		size = fwrite(&(cdl_curvature_wacc[0]), sizeof(double),
				CURVATURE_INDICES_RADIUS, file);
		if (size != CURVATURE_INDICES_RADIUS) {
			fclose(file);
			return CDL_NOK;
		}
		fclose(file);
		return CDL_OK;
	}

	return CDL_NOK;
}

int FileFunctions::saveCurvatureIndexRadius(std::string filename, int cdl_index_radius[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS]) {

	FILE *file;
	int vxi, vyi, wi;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : CDL CURVATURE INDICES Radius\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "VSpaceNumberCellsTransX : %f\n", CDL_TRA_CELLS_CLASSIC);
		fprintf(file, "VSpaceNumberCellsTransY : %f\n", CDL_TRA_CELLS_CLASSIC);
		fprintf(file, "VSpaceNumberCellsRot : %f\n", CDL_ROT_CELLS);

		for (vxi = 0; vxi < CDL_TRA_CELLS_CLASSIC; vxi++) {
			for (vyi = 0; vyi < CDL_TRA_CELLS_CLASSIC; vyi++) {
				for (wi = 0; wi < CDL_ROT_CELLS; wi++) {
					fprintf(file, "%d ", cdl_index_radius[vxi][vyi][wi]);
				}
			fprintf(file, "\n");
			}
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadCurvatureIndexRadius(std::string filename, int cdl_index_radius[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS])
{
  FILE   *file;
  double help;
  int    num,format;
  int    vxi,vyi,wi;

  file=fopen(filename.c_str(),"r");
  if (file!=NULL) {
    // read first line
    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
    if (num!=1) {
      fprintf(stderr,"Error in line 1: unknown file format\n");
      fclose(file);
      return CDL_NOK;
    }

    // read second line
    fscanf(file,"Content : CDL CURVATURE INDICES Radius\n");

    // read third line
    fscanf(file,"Comment : ---\n");

    // read fourth line
    fscanf(file,"VSpaceNumberCellsTransX : %lf\n",&help);
    if ((num!=1) || (help!=CDL_TRA_CELLS_CLASSIC)) {
      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
      fclose(file);
      return CDL_NOK;
    }

    // read fourth line
    fscanf(file,"VSpaceNumberCellsTransY : %lf\n",&help);
    if ((num!=1) || (help!=CDL_TRA_CELLS_CLASSIC)) {
      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
      fclose(file);
      return CDL_NOK;
    }

    // read fifth line
    fscanf(file,"VSpaceNumberCellsRot : %lf\n",&help);
    if ((num!=1) || (help!=CDL_ROT_CELLS)) {
      fprintf(stderr,"Error in line 5: Wrong format or wrong value\n");
      fclose(file);
      return CDL_NOK;
    }

    // read data area
    for (vxi=0;vxi<CDL_TRA_CELLS_CLASSIC;vxi++) {
    	for (vyi=0;vyi<CDL_TRA_CELLS_CLASSIC;vyi++) {
    		for (wi = 0; wi < CDL_ROT_CELLS; wi++) {
    			num=fscanf(file,"%d ",&(cdl_index_radius[vxi][vyi][wi]));
    	        if (num!=1) {
    	        	fprintf(stderr,"Error in data area: Wrong data format\n");
    	        	fclose(file);
    	        	return CDL_NOK;
    	        }
    		}
    		fscanf(file,"\n");
    	}
    }
	fclose(file);
	return CDL_OK;
  }
  return CDL_NOK;
};

int FileFunctions::saveCurvatureIndexICC(std::string filename, int cdl_index_icc[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS]) {

	FILE *file;
	int vxi, vyi, wi;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : CDL CURVATURE INDICES ICC\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "VSpaceNumberCellsTransX : %f\n", CDL_TRA_CELLS_CLASSIC);
		fprintf(file, "VSpaceNumberCellsTransY : %f\n", CDL_TRA_CELLS_CLASSIC);
		fprintf(file, "VSpaceNumberCellsRot : %f\n", CDL_ROT_CELLS);

		for (vxi = 0; vxi < CDL_TRA_CELLS_CLASSIC; vxi++) {
			for (vyi = 0; vyi < CDL_TRA_CELLS_CLASSIC; vyi++) {
				for (wi = 0; wi < CDL_ROT_CELLS; wi++) {
					fprintf(file, "%d ", cdl_index_icc[vxi][vyi][wi]);
				}
				fprintf(file, "\n");
			}
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadCurvatureIndexICC(std::string filename, int cdl_index_icc[CDL_MAX_TRA_CELLS][CDL_MAX_TRA_CELLS][CDL_MAX_ROT_CELLS])
{
  FILE   *file;
  double help;
  int    num,format;
  int    vxi,vyi,wi;

  file=fopen(filename.c_str(),"r");
  if (file!=NULL) {
    // read first line
    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
    if (num!=1) {
      fprintf(stderr,"Error in line 1: unknown file format\n");
      fclose(file);
      return CDL_NOK;
    }

    // read second line
    fscanf(file,"Content : CDL CURVATURE INDICES ICC\n");

    // read third line
    fscanf(file,"Comment : ---\n");

    // read fourth line
    fscanf(file,"VSpaceNumberCellsTransX : %lf\n",&help);
    if ((num!=1) || (help!=CDL_TRA_CELLS_CLASSIC)) {
      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
      fclose(file);
      return CDL_NOK;
    }

    // read fourth line
    fscanf(file,"VSpaceNumberCellsTransY : %lf\n",&help);
    if ((num!=1) || (help!=CDL_TRA_CELLS_CLASSIC)) {
      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
      fclose(file);
      return CDL_NOK;
    }

    // read fifth line
    fscanf(file,"VSpaceNumberCellsRot : %lf\n",&help);
    if ((num!=1) || (help!=CDL_ROT_CELLS)) {
      fprintf(stderr,"Error in line 5: Wrong format or wrong value\n");
      fclose(file);
      return CDL_NOK;
    }

    // read data area
    for (vxi=0;vxi<CDL_TRA_CELLS_CLASSIC;vxi++) {
    	for (vyi=0;vyi<CDL_TRA_CELLS_CLASSIC;vyi++) {
    		for (wi = 0; wi < CDL_ROT_CELLS; wi++) {
    			num=fscanf(file,"%d ",&(cdl_index_icc[vxi][vyi][wi]));
    	        if (num!=1) {
    	        	fprintf(stderr,"Error in data area: Wrong data format\n");
    	        	fclose(file);
    	        	return CDL_NOK;
    	        }
    		}
    		fscanf(file,"\n");
    	}
    }
	fclose(file);
	return CDL_OK;
  }
  return CDL_NOK;
}

int FileFunctions::saveOnlyAngleDist(std::string filename, short cdl_alph_lookup[CDL_MAX_X_CELLS][CDL_MAX_Y_CELLS][CURVATURE_INDICES_RADIUS][CURVATURE_INDICES_ICC_COARSE]) {

	  FILE *file;
	  int counter,size;
	  double a;

	  file = fopen(filename.c_str(),"wb");
	  if (file!=NULL) {
	    counter=0;
	    size=0;

	    a=(double)CDL_V_TRA_MAX_STANDARD;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_V_TRA_MIN_STANDARD;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_V_ROT_MAX;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_V_ROT_MIN;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_V_TRA_STEP;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_V_ROT_STEP;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_TRA_CELLS_CLASSIC;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_ROT_CELLS;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_X_MIN;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_X_MAX;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_Y_MIN;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_Y_MAX;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_RES;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_X_CELLS;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_C_Y_CELLS;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CURVATURE_INDICES_RADIUS;size+=fwrite(&a,sizeof(a),1,file);counter++;
	    a=(double)CDL_TRA_CELLS_COARSE_80;size+=fwrite(&a,sizeof(a),1,file);counter++;

	    if (size != counter) {
	      fclose(file);
	      return CDL_NOK;
	    };

	    size=fwrite(&(cdl_alph_lookup[0][0][0][0]),
	                sizeof(short),
	                CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*CDL_TRA_CELLS_COARSE_80,
	                file);
	    if (size != (CDL_C_X_CELLS*CDL_C_Y_CELLS*CURVATURE_INDICES_RADIUS*CDL_TRA_CELLS_COARSE_80)) {
	      fclose(file);
	      return CDL_NOK;
	    };

	    fclose(file);
	    return CDL_OK;
	  }
	  return CDL_NOK;
}

int FileFunctions::saveRadiusCurvature(std::string filename, double radii[CURVATURE_INDICES_RADIUS]) {

	FILE *file;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : Radius of curvature\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "Curvature Indices : %d\n", CURVATURE_INDICES_RADIUS);

		for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
			fprintf(file, "%f ", radii[i]);
			fprintf(file, "\n");
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadRadiusCurvature(std::string filename, double radii[CURVATURE_INDICES_RADIUS]) {

	  FILE   *file;
	  double help;
	  int    num,format;

	  file=fopen(filename.c_str(),"r");
	  if (file!=NULL) {
	    // read first line
	    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
	    if (num!=1) {
	      fprintf(stderr,"Error in line 1: unknown file format\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read second line
	    fscanf(file,"Content : Radius of curvature\n");

	    // read third line
	    fscanf(file,"Comment : ---\n");

	    // read fourth line
	    fscanf(file,"Curvature Indices : %lf\n",&help);
	    if ((num!=1) || (help!=CURVATURE_INDICES_RADIUS)) {
	      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read data area
	    for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
	    	num=fscanf(file,"%lf ",&(radii[i]));
	    	if (num!=1) {
	    		fprintf(stderr,"Error in data area: Wrong data format\n");
	    	    fclose(file);
	    	    return CDL_NOK;
	    	}
	    	fscanf(file,"\n");
	    }
		fclose(file);
		return CDL_OK;
	  }
	  return CDL_NOK;
}

int FileFunctions::saveRadiusTriple(std::string filename, double radiusOfTriple[CURVATURE_INDICES_RADIUS]) {

	FILE *file;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : Radius of triple\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "Curvature Indices : %d\n", CURVATURE_INDICES_RADIUS);

		for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
			fprintf(file, "%f ", radiusOfTriple[i]);
			fprintf(file, "\n");
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadRadiusTriple(std::string filename, double radiusOfTriple[CURVATURE_INDICES_RADIUS]) {

	  FILE   *file;
	  double help;
	  int    num,format;

	  file=fopen(filename.c_str(),"r");
	  if (file!=NULL) {
	    // read first line
	    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
	    if (num!=1) {
	      fprintf(stderr,"Error in line 1: unknown file format\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read second line
	    fscanf(file,"Content : Radius of triple\n");

	    // read third line
	    fscanf(file,"Comment : ---\n");

	    // read fourth line
	    fscanf(file,"Curvature Indices : %lf\n",&help);
	    if ((num!=1) || (help!=CURVATURE_INDICES_RADIUS)) {
	      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read data area
	    for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
	    	num=fscanf(file,"%lf ",&(radiusOfTriple[i]));
	    	if (num!=1) {
	    		fprintf(stderr,"Error in data area: Wrong data format\n");
	    	    fclose(file);
	    	    return CDL_NOK;
	    	}
	    	fscanf(file,"\n");
	    }
		fclose(file);
		return CDL_OK;
	  }
	  return CDL_NOK;
}

int FileFunctions::saveICCLocationX(std::string filename, boost::multi_array<double, 2> &iccLocationX, int nrICCIndices) {

	FILE *file;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : iccLocationX\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "Curvature Indices : %d\n", nrICCIndices);

		for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
			for (int j = 0; j < nrICCIndices; j++) {
				fprintf(file, "%f ", iccLocationX[i][j]);
			}
			fprintf(file, "\n");
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadICCLocationX(std::string filename, boost::multi_array<double, 2> &iccLocationX, int nrICCIndices) {

	  FILE   *file;
	  double help;
	  int    num,format;

	  file=fopen(filename.c_str(),"r");
	  if (file!=NULL) {
	    // read first line
	    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
	    if (num!=1) {
	      fprintf(stderr,"Error in line 1: unknown file format\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read second line
	    fscanf(file,"Content : iccLocationX\n");

	    // read third line
	    fscanf(file,"Comment : ---\n");

	    // read fourth line
	    fscanf(file,"Curvature Indices : %lf\n",&help);
	    if ((num!=1) || (help!=nrICCIndices)) {
	      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read data area
	    for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
	    	for (int j = 0; j < nrICCIndices; j++) {
	    		num=fscanf(file,"%lf ",&(iccLocationX[i][j]));
	    		if (num!=1) {
	    			fprintf(stderr,"Error in data area: Wrong data format\n");
	    			fclose(file);
	    			return CDL_NOK;
	    		}
	    	}
	    	fscanf(file,"\n");
	    }
		fclose(file);
		return CDL_OK;
	  }
	  return CDL_NOK;
}

int FileFunctions::saveICCLocationY(std::string filename, boost::multi_array<double, 2> &iccLocationY, int nrICCIndices) {

	FILE *file;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : iccLocationY\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "Curvature Indices : %d\n", nrICCIndices);

		for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
			for (int j = 0; j < nrICCIndices; j++) {
				fprintf(file, "%f ", iccLocationY[i][j]);
			}
			fprintf(file, "\n");
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadICCLocationY(std::string filename, boost::multi_array<double, 2> &iccLocationY, int nrICCIndices) {

	  FILE   *file;
	  double help;
	  int    num,format;

	  file=fopen(filename.c_str(),"r");
	  if (file!=NULL) {
	    // read first line
	    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
	    if (num!=1) {
	      fprintf(stderr,"Error in line 1: unknown file format\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read second line
	    fscanf(file,"Content : iccLocationY\n");

	    // read third line
	    fscanf(file,"Comment : ---\n");

	    // read fourth line
	    fscanf(file,"Curvature Indices : %lf\n",&help);
	    if ((num!=1) || (help!=nrICCIndices)) {
	      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read data area
	    for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
	    	for (int j = 0; j < nrICCIndices; j++) {
	    		num=fscanf(file,"%lf ",&(iccLocationY[i][j]));
	    		if (num!=1) {
	    			fprintf(stderr,"Error in data area: Wrong data format\n");
	    			fclose(file);
	    			return CDL_NOK;
	    		}
	    	}
	    	fscanf(file,"\n");
	    }
		fclose(file);
		return CDL_OK;
	  }
	  return CDL_NOK;
}

int FileFunctions::saveICCLocationXT(std::string filename, boost::multi_array<double, 2> &iccLocationXT, int nrICCIndices) {

	FILE *file;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : iccLocationXT\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "Curvature Indices : %d\n", nrICCIndices);

		for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
			for (int j = 0; j < nrICCIndices; j++) {
				fprintf(file, "%f ", iccLocationXT[i][j]);
			}
			fprintf(file, "\n");
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadICCLocationXT(std::string filename, boost::multi_array<double, 2> &iccLocationXT, int nrICCIndices) {

	  FILE   *file;
	  double help;
	  int    num,format;

	  file=fopen(filename.c_str(),"r");
	  if (file!=NULL) {
	    // read first line
	    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
	    if (num!=1) {
	      fprintf(stderr,"Error in line 1: unknown file format\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read second line
	    fscanf(file,"Content : iccLocationXT\n");

	    // read third line
	    fscanf(file,"Comment : ---\n");

	    // read fourth line
	    fscanf(file,"Curvature Indices : %lf\n",&help);
	    if ((num!=1) || (help!=nrICCIndices)) {
	      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read data area
	    for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
	    	for (int j = 0; j < nrICCIndices; j++) {
	    		num=fscanf(file,"%lf ",&(iccLocationXT[i][j]));
	    		if (num!=1) {
	    			fprintf(stderr,"Error in data area: Wrong data format\n");
	    			fclose(file);
	    			return CDL_NOK;
	    		}
	    	}
	    	fscanf(file,"\n");
	    }
		fclose(file);
		return CDL_OK;
	  }
	  return CDL_NOK;
}

int FileFunctions::saveICCLocationYT(std::string filename, boost::multi_array<double, 2> &iccLocationYT, int nrICCIndices) {

	FILE *file;

	file = fopen(filename.c_str(), "w");
	if (file != NULL) {
		fprintf(file, "Format : SFB527 (100)\n");
		fprintf(file, "Content : iccLocationYT\n");
		fprintf(file, "Comment : ---\n");
		fprintf(file, "Curvature Indices : %d\n", nrICCIndices);

		for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
			for (int j = 0; j < nrICCIndices; j++) {
				fprintf(file, "%f ", iccLocationYT[i][j]);
			}
			fprintf(file, "\n");
		}
		fclose(file);
		return CDL_OK;
	}
	return CDL_NOK;
}

int FileFunctions::loadICCLocationYT(std::string filename, boost::multi_array<double, 2> &iccLocationYT, int nrICCIndices) {

	  FILE   *file;
	  double help;
	  int    num,format;

	  file=fopen(filename.c_str(),"r");
	  if (file!=NULL) {
	    // read first line
	    num=fscanf(file,"Format : SFB527 (%d)\n",&format);
	    if (num!=1) {
	      fprintf(stderr,"Error in line 1: unknown file format\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read second line
	    fscanf(file,"Content : iccLocationYT\n");

	    // read third line
	    fscanf(file,"Comment : ---\n");

	    // read fourth line
	    fscanf(file,"Curvature Indices : %lf\n",&help);
	    std::cout << "help: " << help << std::endl;
	    if ((num!=1) || (help!=nrICCIndices)) {
	      fprintf(stderr,"Error in line 4: Wrong format or wrong value\n");
	      fclose(file);
	      return CDL_NOK;
	    }

	    // read data area
	    for (int i = 0; i < CURVATURE_INDICES_RADIUS; i++) {
	    	for (int j = 0; j < nrICCIndices; j++) {
	    		num=fscanf(file,"%lf ",&(iccLocationYT[i][j]));
	    		//std::cout << iccLocationYT[i][j] << std::endl;
	    		if (num!=1) {
	    			fprintf(stderr,"Error in data area: Wrong data format\n");
	    			fclose(file);
	    			return CDL_NOK;
	    		}
	    	}
	    	fscanf(file,"\n");
	    }
		fclose(file);
		return CDL_OK;
	  }
	  return CDL_NOK;
}
