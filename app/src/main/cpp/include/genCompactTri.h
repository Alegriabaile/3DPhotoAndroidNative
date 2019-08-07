//
// Created by ale on 19-3-24.
//

#ifndef I3D_GENCOMPACTTRI_H
#define I3D_GENCOMPACTTRI_H
#include "i3d.h"
#include "Frame.h"

//generate triangles from front and back layers, with simplified processing.
//then write the information to .obj&.mtl files.
int genCompactTri(const std::string& outputDir, i3d::Frame& pano, const int icount=30);

int genCompactTri(i3d::Frame& pano, cv::Mat & f_b_texture, std::vector<float>& f_b_vertices, const int icount=30);
#endif //I3D_GENCOMPACTTRI_H
