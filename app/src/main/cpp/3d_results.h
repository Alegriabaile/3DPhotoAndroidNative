//
// Created by ale on 19-8-8.
//

#ifndef NATIVE0701_3D_RESULTS_H
#define NATIVE0701_3D_RESULTS_H

#include "i3d.h"
#include <vector>

//f_b_vertices:
// vertice0 = [x0,y0,z0,u0,v0].
// triangle0 = [vertices0, vertices1, vertices2].
// num_of_triangles = f_b_vertices.size()/(5*3)
std::vector<float> vertices;

//color texture;
cv::Mat texture;

//input frames and intermediate results.
std::vector<i3d::Frame> frames;

#endif //NATIVE0701_3D_RESULTS_H
