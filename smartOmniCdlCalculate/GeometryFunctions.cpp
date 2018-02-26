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
#include "GeometryFunctions.h"
#include <cmath>

double min(double a,double b)
{
  if (a<b) return a; else return b;
}

int GeometryFunctions::cdl_calc_distance(double px,double py, double xm, double ym,
                      double ax,double ay,double bx,double by,
                      int trans,int rot,
                      double *distance,double *angle)
{
  double   a,b,c; 		// different representation of line AB
  double   s1x,s1y,s2x,s2y;	// intersection of circle with line AB
  int      number;		// number of intersections
  double   arc;                 // arc length of circle segment
  double   alpha;               // alpha of circle segment
  double   r;                   // radius of circle with m through p

  double   intersection_x,intersection_y,point_x,point_y;
  int      status;

  status = cdl_ab_axbyc(ax,ay,bx,by,&a,&b,&c);
  status = cdl_calc_circle_radius(xm,ym,px,py,&r);
  status = cdl_intersection_circle_line(xm,ym,r,a,b,c,&s1x,&s1y,&s2x,&s2y,&number);

  *distance = min(CDL_MAX_DISTANCE,fabs(2*M_PI*sqrt(ym*ym+xm*xm)));
  *angle    = 2*M_PI;

  if (number == 0) {
    // no intersection
    *distance = min(CDL_MAX_DISTANCE,*distance);
    *angle    = min(2*M_PI,*angle);
  }
  if (number > 0) {
    // at least one intersection
    status = cdl_check_point_on_line(s1x,s1y,ax,ay,bx,by);
    if (status==CDL_OK) {
      // calculated intersection is true intersection
      // now we have obstacle point p and intersection s1 and have to
      // calculate the distance when moving along the arc

      // because the angle calculation assumes the origin for rotation
      // in (0,0), we have to transform the coordinates first.
      intersection_x = s1x - xm;
      intersection_y = s1y - ym;
      point_x        = px - xm;
      point_y        = py - ym;
      alpha          = cdl_calc_angle(intersection_x,intersection_y,point_x,point_y,rot);

      // ---------------------------------------------------------------------
      // now alpha contains the remaining rotating angle until collision
      // with obstacle point. Now we have to compute the remaining translation
      // distance with respect to the center point of the robot.
      // ---------------------------------------------------------------------
      arc = alpha * sqrt(ym*ym+xm*xm);
      if (arc < 0.0) arc *= -1.0;        // to ensure positive arc length
      *distance = min(arc,*distance);
      if (alpha < 0.0) alpha *= -1.0;    // to ensure positive angle of circle segment
      *angle    = min(alpha,*angle);
    } else {
      // intersection point 1 not on line
    }
  }
  if (number > 1) {
    // there is another intersection
    status = cdl_check_point_on_line(s2x,s2y,ax,ay,bx,by);
    if (status==CDL_OK) {
      // calculated intersection is true intersection, same procedure as before
      intersection_x = s2x - xm;
      intersection_y = s2y - ym;
      point_x        = px - xm;
      point_y        = py - ym;
      alpha          = cdl_calc_angle(intersection_x,intersection_y,point_x,point_y,rot);

      // compare to the above case ...
      arc   = alpha * sqrt(ym*ym+xm*xm);
      if (arc < 0.0) arc *= -1.0;	// to ensure positive arc length
      *distance = min(arc,*distance);
      if (alpha < 0.0) alpha *= -1.0;   // to ensure positive angle of circle segment
      *angle    = min(alpha,*angle);
    } else {
      // intersection point 2 not on line
    }
  }
  return CDL_OK;
}

int GeometryFunctions::cdl_calc_distance_straight_sideway(double px,double py,
                               double ax,double ay,double bx,double by,
                               int trans,double *distance)
{
  double   a1,b1,c1,a2,b2,c2;
  double   sx,sy;
  double   hx,hy;
  double   dist1,dist2;
  int      status;

  // first sort the line end points with respect to x-axis
  if (bx > ax) {
    hx = bx;bx = ax;ax = hx;
    hy = by;by = ay;ay = hy;
  };

  if (ax==bx) {
	// line is parallel to y-axis
	  if (ax==px) {
		      // point p in line with line ab
		      if (trans>0) {
		        // move left
		        dist1 = py-ay;
		        dist2 = py-by;
		        if ((dist1>=0.0) && (dist2>=0.0)) {
		          // there is some space until collision
		          *distance = min(dist1,dist2);
		        } else if((dist1<0.0) && (dist2<0.0)) {
		          // there is some space, but wrong direction, so no collision
		          *distance = CDL_MAX_DISTANCE;
		        } else {
		          // p is on line ab
		          // collision distance zero
		          *distance = 0.0;
		        }
		      } else {
		        // move right
		        dist1 = ay-py;
		        dist2 = by-py;
		        if ((dist1>=0.0) && (dist2>=0.0)) {
		          // there is some space until collision
		          *distance = min(dist1,dist2);
		        } else if ((dist1<0.0) && (dist2<0.0)) {
		          // there is some space, but wrong direction, so no collision
		          *distance = CDL_MAX_DISTANCE;
		        } else {
		          // p is on line ab
		          // collision distance zero
		          *distance = 0.0;
		        }
		      }
		    }

	  else {
	        // point p not in line with ab
	        // no collision
	        *distance = CDL_MAX_DISTANCE;
	      }

  } else {

	  if ((ax>=px) && (px>=bx)) {
	  		// collision can occure
	  	    // second line is in parallel to y-axis with px
	  	    status = GeometryFunctions::cdl_ab_axbyc(ax,ay,bx,by,&a1,&b1,&c1);
	  	    status = GeometryFunctions::cdl_ab_axbyc(px,py,px,py+1.0,&a2,&b2,&c2);
	  	    status = GeometryFunctions::cdl_line_intersection(a1,b1,-c1,a2,b2,-c2,&sx,&sy);

	  	// IN DIESEM FALL LIEFERT FKT IMMER EINEN SCHNITTPUNKT
	  	      if (status != CDL_OK) std::cout << "ERROR\n";

	  	      if (trans>0) {
	  	        // move left
	  	        dist1 = py-sy;
	  	        if (dist1>=0.0) {
	  	          // there is some space until collision
	  	          *distance = dist1;
	  	        } else {
	  	          // no collision can occur
	  	          *distance = CDL_MAX_DISTANCE;
	  	        }
	  	      } else {
	  	        // move right
	  	        dist1 = sy-py;
	  	        if (dist1>=0.0) {
	  	          // there is some space until collision
	  	          *distance = dist1;
	  	        } else {
	  	          // no collision can occur
	  	          *distance = CDL_MAX_DISTANCE;
	  	        }
	  	      }
	  	    } else {
	  	      // no collision possible
	  	      *distance = CDL_MAX_DISTANCE;
	  	    }
  }
  return CDL_OK;
}

int GeometryFunctions::cdl_calc_distance_straight(double px,double py,
                               double ax,double ay,double bx,double by,
                               int trans,double *distance)
{
  double   a1,b1,c1,a2,b2,c2;
  double   sx,sy;
  double   hx,hy;
  double   dist1,dist2;
  int      status;

  // first sort the line end points with respect to y-axis
  if (by < ay) {
    hx = bx;bx = ax; ax = hx;
    hy = by;by = ay;ay = hy;
  };
  if (ay==by) {
    // line is parallel to x-axis
    if (ay==py) {
      // point p in line with line ab
      if (trans>0) {
        // move forward
        dist1 = px-ax;
        dist2 = px-bx;
        if ((dist1>=0.0) && (dist2>=0.0)) {
          // there is some space until collision
          *distance = min(dist1,dist2);
        } else if((dist1<0.0) && (dist2<0.0)) {
          // there is some space, but wrong direction, so no collision
          *distance = CDL_MAX_DISTANCE;
        } else {
          // p is on line ab
          // collision distance zero
          *distance = 0.0;
        }
      } else {
        // move backward
        dist1 = ax-px;
        dist2 = bx-px;
        if ((dist1>=0.0) && (dist2>=0.0)) {
          // there is some space until collision
          *distance = min(dist1,dist2);
        } else if ((dist1<0.0) && (dist2<0.0)) {
          // there is some space, but wrong direction, so no collision
          *distance = CDL_MAX_DISTANCE;
        } else {
          // p is on line ab
          // collision distance zero
          *distance = 0.0;
        }
      }
    } else {
      // point p not in line with ab
      // no collision
      *distance = CDL_MAX_DISTANCE;
    }
  } else {
    // line is not parallel to x-axis
    if ((ay<=py) && (py<=by)) {
      // collision can occure
      // second line is in parallel to x-axis with py
      status = cdl_ab_axbyc(ax,ay,bx,by,&a1,&b1,&c1);
      status = cdl_ab_axbyc(px,py,px+1.0,py,&a2,&b2,&c2);
      status = cdl_line_intersection(a1,b1,-c1,a2,b2,-c2,&sx,&sy);

// IN DIESEM FALL LIEFERT FKT IMMER EINEN SCHNITTPUNKT
      if (status != CDL_OK) std::cout << "ERROR\n" << std::endl;

      if (trans>0) {
        // move forward
        dist1 = px-sx;
        if (dist1>=0.0) {
          // there is some space until collision
          *distance = dist1;
        } else {
          // no collision can occur
          *distance = CDL_MAX_DISTANCE;
        }
      } else {
        // move backward
        dist1 = sx-px;
        if (dist1>=0.0) {
          // there is some space until collision
          *distance = dist1;
        } else {
          // no collision can occur
          *distance = CDL_MAX_DISTANCE;
        }
      }
    } else {
      // no collision possible
      *distance = CDL_MAX_DISTANCE;
    }
  }
  return CDL_OK;
}

// -----------------------------------------------------------------
// Calculates intersection of two lines if available
//
// input lines are assumed as ax+by+c=0
// input    a1,b1,c1         Parameter line 1
// input    a2,b2,c2         Parameter line 2
// output   x,y              line intersection point
// output   status  CDL_OK   found intersection
//                  CDL_INF  lines are equal, infinity intersections
//                  CDL_NO   lines are parallel, no intersection
// -----------------------------------------------------------------
int GeometryFunctions::cdl_line_intersection(double a1,double b1,double c1,
                          double a2,double b2,double c2,
                          double *x,double *y)
{
  int status;

  if ((a1*b2-a2*b1) == 0.0) {
    // parallel lines
    status = CDL_NO;
  } else if ((a2==0.0) && (b2==0.0) && (c2==0.0)) {
    // equal lines
    status = CDL_INF;
  } else if ((a1==0.0) && (b1==0.0) && (c1==0.0)) {
    // equal lines
    status = CDL_INF;
  } else if ((a1/a2==b1/b2) && (a1/a2==c1/c2)) {
    // equal lines
    status = CDL_INF;
  } else {
    // lines not parallel
    *x = (b1*c2-b2*c1)/(a1*b2-a2*b1);
    *y = (c1*a2-c2*a1)/(a1*b2-a2*b1);
    status = CDL_OK;
  }
  return status;
}

int GeometryFunctions::cdl_ab_axbyc(double pax,double pay,double pbx,double pby,
									double *a,double *b,double *c)
{
	double      deltax,deltay;

	deltax = pbx - pax;
	deltay = pby - pay;
	if (fabs(deltax) < CDL_ACCURACY) {
	  // parallel to y
	  *a = 1.0;
	  *b = 0.0;
	  *c = pax;
	} else if (fabs(deltay) < CDL_ACCURACY) {
	  // parallel to x
	  *a = 0.0;
	  *b = 1.0;
	  *c = pay;
	} else if ((fabs(deltay) < CDL_ACCURACY) && (fabs(deltax) < CDL_ACCURACY)) {
	  // distance A <-> B too small
	  return CDL_NOK;
	} else {
	  *a = -deltay/deltax;
	  *b = 1.0;
	  *c = *a * pax + *b * pay;
	}
	 return CDL_OK;
}

double GeometryFunctions::cdl_calc_angle(double xs,double ys,double xe,double ye,int direction)
{
  double diff;

  diff = atan2(ye,xe) - atan2(ys,xs);
  // calculation for mathematically positive rotation direction
  if (diff < 0) diff += 2*M_PI;
  if (direction < 0) {
    // calculation for mathematically negative rotation direction
    diff =  -(2*M_PI - diff);
    if (diff <= -2*M_PI) diff += 2*M_PI;
  }
  return diff;
}

int GeometryFunctions::cdl_calc_circle_radius(double mx,double my,double px,double py,double *r)
{
  *r = sqrt((px-mx)*(px-mx)+(py-my)*(py-my));
	  return CDL_OK;
}

int GeometryFunctions::cdl_intersection_circle_line(double xm,double ym,double r,
                                        double a,double b,double c,
	                                    double *x1, double *y1,double *x2,double *y2,
	                                    int *ns)
{
  double      ab2;
  double      dis;
  double      cc;
  double      wu;

  *ns  = 0;
  ab2 = a*a + b*b;
  if (ab2 != 0.0) {
    cc = c - a*xm - b*ym;
    dis = r*r*ab2 - cc*cc;
    if (dis >= 0.0) {
    *ns = 2;
    wu = sqrt(dis);
    if (wu == 0.0) *ns=1;
    /* ----- damit stets x1 <= x2 gilt ----- */
    if (b >= 0.0) {
    *x1 = xm + (a*cc-b*wu)/ab2;
    *y1 = ym + (b*cc+a*wu)/ab2;
    *x2 = xm + (a*cc+b*wu)/ab2;
    *y2 = ym + (b*cc-a*wu)/ab2;
  } else {
    *x2 = xm + (a*cc-b*wu)/ab2;
    *y2 = ym + (b*cc+a*wu)/ab2;
    *x1 = xm + (a*cc+b*wu)/ab2;
    *y1 = ym + (b*cc-a*wu)/ab2;
    }
  }
}
  return CDL_OK;
}

int GeometryFunctions::cdl_check_point_on_line(double px,double py,
                                   double ax,double ay,double bx,double by)
{
	double t;
    double y;
    double EPS = 2.22044604925031e-16;

	if ((bx-ax)==0.0) {
	   // line parallel to y-axis
	   t = (py-ay)/(by-ay);
	   if ((fabs(px-ax) < CDL_ACCURACY) && (0.0<=t) && (t<=1.0)) {
	     return CDL_OK;
	   }
	} else {
	   t = (px-ax)/(bx-ax);
	   y = ay + t*(by-ay);
	   if ((0.0<=t) && (t<=1.0) && (fabs(y-py)<CDL_ACCURACY)) {
	     return CDL_OK;
	   }
	}

	return CDL_NOK;
}

int GeometryFunctions::cdl_check_point_inside_polygon(double x_p,double y_p,
                                   struct cdl_polygon_struct *polygon)
{
  int     counter;
  double  x_help,y_help;
  double  t;
  double  x;
  int     i;

  counter=0;
  for (i=0;i<polygon->number_of_segments;i++) {
    if (polygon->line[i].y1 != polygon->line[i].y2) {
      if (polygon->line[i].y1 > polygon->line[i].y2) {
        x_help=polygon->line[i].x1;
        polygon->line[i].x1=polygon->line[i].x2;
        polygon->line[i].x2=x_help;
        y_help=polygon->line[i].y1;
        polygon->line[i].y1=polygon->line[i].y2;
        polygon->line[i].y2=y_help;
      }
      t=(y_p-polygon->line[i].y1)/(polygon->line[i].y2-polygon->line[i].y1);
      x=polygon->line[i].x1+t*(polygon->line[i].x2-polygon->line[i].x1);
      if ((0<=t) && (t<1.0) && (x<=x_p)) counter++;
    }
  }
  if ((counter%2)==0) return CDL_NOK; else return CDL_OK;
}

void GeometryFunctions::cdl_calc_distance_linear(double x, double y, double s, double ax, double ay, double bx, double by, double trans_x, double trans_y, double *dist) {

	double a1 = 0.0;
	double b1 = 0.0;
	double c1 = 0.0;
	double a2 = 0.0;
	double b2 = 0.0;
	double c2 = 0.0;
	double sx = 0.0;
	double sy = 0.0;
	int status = 0;

	// 1. Define line through obstacle(x,y) with slope s

    double helpX = x + sin(s);
	double helpY = y + cos(s);

	GeometryFunctions::cdl_ab_axbyc(x,y,helpX,helpY,&a1,&b1,&c1);
	GeometryFunctions::cdl_ab_axbyc(ax,ay,bx,by,&a2,&b2,&c2);

	// 2. Does line intersect current segment ?
	status = GeometryFunctions::cdl_line_intersection(a1,b1,-c1,a2,b2,-c2,&sx,&sy);

	if (status == CDL_OK) {
		// Yes, intersects => Distance = euclidean distance between intersection point with segment and obstacle
		if (GeometryFunctions::cdl_check_point_on_line(sx,sy,ax,ay,bx,by) == CDL_OK) {

			*dist = sqrt(pow((x-sx),2) + pow((y-sy),2));

			if (trans_x >= 0 && trans_y >= 0 && (x <= 0 && y <= 0)) {
				*dist = CDL_MAX_DISTANCE;
			}

			else if (trans_x >= 0 && trans_y <= 0 && (x <= 0 && y >= 0)) {
				*dist = CDL_MAX_DISTANCE;
			}

			else if (trans_x <= 0 && trans_y >= 0 && (x >= 0 && y <= 0)) {
				*dist = CDL_MAX_DISTANCE;
			}

			else if (trans_x <= 0 && trans_y <= 0 && (x >= 0 && y >= 0)) {
				*dist = CDL_MAX_DISTANCE;
			}
		}

		else {
			// No, does not intersect => Distance = CDL_MAX_DISTANCE
			*dist = CDL_MAX_DISTANCE;
		}
	}
	else {
		// No, does not intersect => Distance = CDL_MAX_DISTANCE
		*dist = CDL_MAX_DISTANCE;
	}
}
