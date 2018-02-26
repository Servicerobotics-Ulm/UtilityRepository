
#ifndef _CONTOUR_TRACTOR_H
#define _CONTOUR_TRACTOR_H

#include "smartCdlTypes.hh"

void definePolygon_Tractor(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "Tractor";
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

// Tractor robot shape
polygon.line[0].x1 =  1900.0;polygon.line[0].y1 =  1200.0;  // AB
polygon.line[0].x2 =  1900.0;polygon.line[0].y2 = -1200.0;

polygon.line[1].x1 =  1900.0;polygon.line[1].y1 = -1200.0;  // BC
polygon.line[1].x2 =  -1900.0;polygon.line[1].y2 = -1200.0;

polygon.line[2].x1 =  -1900.0;polygon.line[2].y1 = -1200.0;  // CD
polygon.line[2].x2 = -1900.0;polygon.line[2].y2 = 1200.0;

polygon.line[3].x1 = -1900.0;polygon.line[3].y1 = 1200.0;  // DA
polygon.line[3].x2 = 1900.0;polygon.line[3].y2 = 1200.0;
}
#endif
