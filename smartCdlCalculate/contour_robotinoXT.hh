
#ifndef _CONTOUR_ROBOTINOXT_H
#define _CONTOUR_ROBOTINOXT_H

#include "smartCdlTypes.hh"

void definePolygon_RobotinoXT(cdl_polygon_struct &polygon, std::string &postfix){

postfix = "RobotinoXT";
polygon.number_of_segments = 10;

// hoerger date="09.01.2012"
// robot shape RobotinoXT
  // -------------------------------------------------------
  //
  //      A _____ B
  //        |   |
  //        |   |
  //      J |   | C                ^ x
  //       /    \                  |
  //    I /      \ D               |
  //     |        |     y <--------+--
  //     |        |
  //    H \      / E
  //       \____/
  //       G     F
  // -------------------------------------------------------

  polygon.line[0].x1 =  495.0;polygon.line[0].y1 =   65.0;  // AB
  polygon.line[0].x2 =  495.0;polygon.line[0].y2 =  -65.0;

  polygon.line[1].x1 =  495.0;polygon.line[1].y1 =  -65.0;  // BC
  polygon.line[1].x2 =  185.0;polygon.line[1].y2 =  -65.0;

  polygon.line[2].x1 =  185.0;polygon.line[2].y1 =  -65.0;  // CD
  polygon.line[2].x2 =   92.5;polygon.line[2].y2 = -185.0;

  polygon.line[3].x1 =   92.5;polygon.line[3].y1 = -185.0;  // DE
  polygon.line[3].x2 =  -92.5;polygon.line[3].y2 = -185.0;

  polygon.line[4].x1 =  -92.5;polygon.line[4].y1 = -185.0;  // EF
  polygon.line[4].x2 = -185.0;polygon.line[4].y2 =  -65.0;

  polygon.line[5].x1 = -185.0;polygon.line[5].y1 =  -65.0;  // FG
  polygon.line[5].x2 = -185.0;polygon.line[5].y2 =   65.0;

  polygon.line[6].x1 = -185.0;polygon.line[6].y1 =   65.0;  // GH
  polygon.line[6].x2 =  -92.5;polygon.line[6].y2 =  185.0;

  polygon.line[7].x1 =  -92.5;polygon.line[7].y1 =  185.0;  // HI
  polygon.line[7].x2 =   92.5;polygon.line[7].y2 =  185.0;
  
  polygon.line[8].x1 =   92.5;polygon.line[8].y1 =  185.0;  // IJ
  polygon.line[8].x2 =  185.0;polygon.line[8].y2 =   65.0;
  
  polygon.line[9].x1 =  185.0;polygon.line[9].y1 =   65.0;  // JA
  polygon.line[9].x2 =  495.0;polygon.line[9].y2 =   65.0;

}
#endif
