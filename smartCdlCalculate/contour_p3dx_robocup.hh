
#ifndef _CONTOUR_P3DX_ROBOCUP_H
#define _CONTOUR_P3DX_ROBOCUP_H

#include "smartCdlTypes.hh"

void definePolygon_P3DX_Robocup(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "P3DXrobocup";
polygon.number_of_segments = 7;

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


// robot shape P3DX --- robocup setup
  polygon.line[0].x1 =  220.0;polygon.line[0].y1 =  140.0;  // AB
  polygon.line[0].x2 =  220.0;polygon.line[0].y2 = -140.0;

  polygon.line[1].x1 =  220.0;polygon.line[1].y1 = -140.0;  // BC
  polygon.line[1].x2 =   60.0;polygon.line[1].y2 = -300.0;

  polygon.line[2].x1 =   60.0;polygon.line[2].y1 = -300.0;  // CD
  polygon.line[2].x2 = -180.0;polygon.line[2].y2 = -300.0;

  polygon.line[3].x1 = -180.0;polygon.line[3].y1 = -300.0;  // DE
  polygon.line[3].x2 = -310.0;polygon.line[3].y2 = -140.0;

  polygon.line[4].x1 =  -310.0;polygon.line[4].y1 = -140.0;  // EF
  polygon.line[4].x2 =  -310.0;polygon.line[4].y2 =  140.0;

  polygon.line[5].x1 =  -310.0;polygon.line[5].y1 = 140.0;  // FG
  polygon.line[5].x2 =  -180.0;polygon.line[5].y2 = 300.0;

  polygon.line[6].x1 =  -180.0;polygon.line[6].y1 = 300.0;  // GH
  polygon.line[6].x2 =    60.0;polygon.line[6].y2 = 300.0;

  polygon.line[6].x1 =    60.0;polygon.line[6].y1 = 300.0;  // HA
  polygon.line[6].x2 =   220.0;polygon.line[6].y2 = 140.0;

}
#endif
