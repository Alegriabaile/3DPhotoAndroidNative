//
// Created by ale on 19-3-20.
//

#ifndef I3D_GENOBJFROMSIMPLIFIEDFB_H
#define I3D_GENOBJFROMSIMPLIFIEDFB_H

#include "i3d.h"
#include "Frame.h"

//generate triangles from front and back layers, with simplified processing.
//then write the information to .obj&.mtl files.
int genSimplifiedTri(const std::string& outputDir, i3d::Frame& pano, const int icount=30);

#endif //I3D_GENOBJFROMSIMPLIFIEDFB_H
