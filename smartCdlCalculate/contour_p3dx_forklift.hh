
#ifndef _CONTOUR_P3DX_FORKLIFT_H
#define _CONTOUR_P3DX_FORKLIFT_H

#include "smartCdlTypes.hh"

void definePolygon_P3DX_Forklift(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "P3DXforklift";
polygon.number_of_segments = 10;

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


// robot shape P3DX with some safety clearance and Forklift
  polygon.line[0].x1 =  450.0;polygon.line[0].y1 =  100.0;  // AB
  polygon.line[0].x2 =  450.0;polygon.line[0].y2 = -100.0;

  polygon.line[1].x1 =  450.0;polygon.line[1].y1 = -100.0;  // BC
  polygon.line[1].x2 =   60.0;polygon.line[1].y2 = -200.0;

  polygon.line[2].x1 =   60.0;polygon.line[2].y1 = -200.0;  // CD
  polygon.line[2].x2 =  -90.0;polygon.line[2].y2 = -200.0;

  polygon.line[3].x1 =  -90.0;polygon.line[3].y1 = -200.0;  // DE
  polygon.line[3].x2 = -230.0;polygon.line[3].y2 = -175.0;

  polygon.line[4].x1 =  -230.0;polygon.line[4].y1 = -175.0;  // EF
  polygon.line[4].x2 =  -270.0;polygon.line[4].y2 =  -70.0;

  polygon.line[5].x1 =  -270.0;polygon.line[5].y1 = -70.0;  // FG
  polygon.line[5].x2 =  -270.0;polygon.line[5].y2 =  70.0;

  polygon.line[6].x1 =  -270.0;polygon.line[6].y1 =  70.0;  // GH
  polygon.line[6].x2 =  -230.0;polygon.line[6].y2 = 175.0;

  polygon.line[7].x1 =  -230.0;polygon.line[7].y1 = 175.0;  // HI
  polygon.line[7].x2 =   -90.0;polygon.line[7].y2 = 200.0;

  polygon.line[8].x1 =   -90.0;polygon.line[8].y1 = 200.0;  // IJ
  polygon.line[8].x2 =    60.0;polygon.line[8].y2 = 200.0;

  polygon.line[9].x1 =    60.0;polygon.line[9].y1 = 200.0;  // JA
  polygon.line[9].x2 =   450.0;polygon.line[9].y2 = 100.0;

}
#endif
