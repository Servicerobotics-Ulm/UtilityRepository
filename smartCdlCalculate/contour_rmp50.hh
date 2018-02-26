
#ifndef _CONTOUR_RMP50_H
#define _CONTOUR_RMP50_H

#include "smartCdlTypes.hh"

void definePolygon_RMP50(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "RMP50";
polygon.number_of_segments = 8;

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

 // LARRY robot shape RMP50

  polygon.line[0].x1 =  327.0;polygon.line[0].y1 =  143.0;  // AB
  polygon.line[0].x2 =  327.0;polygon.line[0].y2 = -143.0;

  polygon.line[1].x1 =  327.0;polygon.line[1].y1 = -143.0;  // BC
  polygon.line[1].x2 =  205.0;polygon.line[1].y2 = -299.0;

  polygon.line[2].x1 =  205.0;polygon.line[2].y1 = -299.0;  // CD
  polygon.line[2].x2 = -205.0;polygon.line[2].y2 = -299.0;

  polygon.line[3].x1 = -205.0;polygon.line[3].y1 = -299.0;  // DE
  polygon.line[3].x2 = -327.0;polygon.line[3].y2 = -143.0;

  polygon.line[4].x1 = -327.0;polygon.line[4].y1 = -143.0;  // EF
  polygon.line[4].x2 = -327.0;polygon.line[4].y2 =  143.0;

  polygon.line[5].x1 = -327.0;polygon.line[5].y1 =  143.0;  // FG
  polygon.line[5].x2 = -205.0;polygon.line[5].y2 =  299.0;

  polygon.line[6].x1 = -205.0;polygon.line[6].y1 =  299.0;  // GH
  polygon.line[6].x2 =  205.0;polygon.line[6].y2 =  299.0;

  polygon.line[7].x1 =  205.0;polygon.line[7].y1 =  299.0;  // HA
  polygon.line[7].x2 =  327.0;polygon.line[7].y2 =  143.0; 

//TODO if usefull move to new contour
// TEST larry 4 corners
/*
  polygon.line[0].x1 =  349.0;polygon.line[0].y1 = -299.0;
  polygon.line[0].x2 = -349.0;polygon.line[0].y2 = -299.0;


  polygon.line[1].x1 = -349.0;polygon.line[1].y1 = -299.0;
  polygon.line[1].x2 = -349.0;polygon.line[1].y2 =  299.0;

  polygon.line[2].x1 = -349.0;polygon.line[2].y1 =  299.0;  
  polygon.line[2].x2 =  349.0;polygon.line[2].y2 =  299.0;

  polygon.line[3].x1 =  349.0;polygon.line[3].y1 =  299.0;  
  polygon.line[3].x2 =  349.0;polygon.line[3].y2 = -299.0;
*/

}
#endif
