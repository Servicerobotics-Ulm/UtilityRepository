
#ifndef _CONTOUR_ROBOTINO3CELL25MM_H
#define _CONTOUR_ROBOTINO3CELL25MM_H

#include "smartCdlTypes.hh"

void definePolygon_Robotino3Cell25mm(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "Robotino3Cell25mm";

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


  polygon.number_of_segments = 8;


// robotino 3

  polygon.line[0].x1 =  250.0;polygon.line[0].y1 =   124.0;  // AB
  polygon.line[0].x2 =  250.0;polygon.line[0].y2 =  -124.0;

  polygon.line[1].x1 =  250.0;polygon.line[1].y1 =  -124.0;  // BC
  polygon.line[1].x2 =  125.0;polygon.line[1].y2 =  -249.0;

  polygon.line[2].x1 =  125.0;polygon.line[2].y1 =  -249.0;  // CD
  polygon.line[2].x2 = -125.0;polygon.line[2].y2 =  -249.0;

  polygon.line[3].x1 = -125.0;polygon.line[3].y1 =  -249.0;  // DE
  polygon.line[3].x2 = -250.0;polygon.line[3].y2 =  -124.0;

  polygon.line[4].x1 =  -250.0;polygon.line[4].y1 = -124.0;  // EF
  polygon.line[4].x2 =  -250.0;polygon.line[4].y2 =  124.0;

  polygon.line[5].x1 =  -250.0;polygon.line[5].y1 =  124.0;  // FG
  polygon.line[5].x2 =  -125.0;polygon.line[5].y2 =  249.0;

  polygon.line[6].x1 =  -125.0;polygon.line[6].y1 =  249.0;  // GH
  polygon.line[6].x2 =   125.0;polygon.line[6].y2 =  249.0;

  polygon.line[7].x1 =   125.0;polygon.line[7].y1 =  249.0;  // HA
  polygon.line[7].x2 =   250.0;polygon.line[7].y2 =  124.0;

}
#endif
