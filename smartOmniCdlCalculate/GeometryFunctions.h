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

#include "smartCdlCons.h"
#include "smartCdlTypes.h"

class GeometryFunctions {

public:

	static int cdl_ab_axbyc(double pax,double pay,double pbx,double pby,
	                 double *a,double *b,double *c);

	static double cdl_calc_angle(double xs,double ys,double xe,double ye,int direction);

	static int cdl_calc_circle_radius(double mx,double my,double px,double py,double *r);

	static int cdl_intersection_circle_line(double xm,double ym,double r,
	                                 double a,double b,double c,
	                                 double *x1, double *y1,double *x2,double *y2,
	                                 int *ns);

	static int cdl_check_point_on_line(double px,double py, double ax,double ay,double bx,double by);

	static int cdl_check_point_inside_polygon(double x_p,double y_p, struct cdl_polygon_struct *polygon);

	static int cdl_line_intersection(double a1,double b1,double c1,
	                          double a2,double b2,double c2,
	                          double *x,double *y);

	static int cdl_calc_distance_straight(double px,double py,
	                               double ax,double ay,double bx,double by,
	                               int trans,double *distance);

	static int cdl_calc_distance_straight_sideway(double px,double py,
	                               double ax,double ay,double bx,double by,
	                               int trans,double *distance);

	static int cdl_calc_distance(double px,double py,double xm, double ym,
	                      double ax,double ay,double bx,double by,
	                      int trans,int rot,
	                      double *distance,double *angle);

	static void cdl_calc_distance_linear(double x, double y, double s, double ax, double ay, double bx, double by, double trans_x, double trans_y, double *dist);
};
