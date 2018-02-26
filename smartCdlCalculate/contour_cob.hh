
#ifndef _CONTOUR_COB_H
#define _CONTOUR_COB_H

#include "smartCdlTypes.hh"

void definePolygon_COB(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "COB";
polygon.number_of_segments = 4;

  // -------------------------------------------------------
  //      A ____ B                 ^ x
  //       /    \                  |
  //    H /      \ C               |
  //     |        |     y <--------+--
  //     |        |
  //    G \      / D
  //       \____/
  //       F     E
  // -------------------------------------------------------

// robot shape for Frauenhofer CareOBot with safety clearance (Width=550+100 Depth=750+100)
  polygon.line[0].x1 =  425.0;polygon.line[0].y1 =  325.0;  // AB
  polygon.line[0].x2 =  425.0;polygon.line[0].y2 = -325.0;

  polygon.line[1].x1 =  425.0;polygon.line[1].y1 = -325.0;  // BC
  polygon.line[1].x2 =  -425.0;polygon.line[1].y2 = -325.0;

  polygon.line[2].x1 =  -425.0;polygon.line[2].y1 = -325.0;  // CD
  polygon.line[2].x2 =  -425.0;polygon.line[2].y2 = 325.0;

  polygon.line[3].x1 = -425.0;polygon.line[3].y1 = 325.0;  // DA
  polygon.line[3].x2 = 425.0;polygon.line[3].y2 = 325.0;
 
}
#endif
