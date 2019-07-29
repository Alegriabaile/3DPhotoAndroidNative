//
// Created by ale on 18-12-20.
//

#ifndef I3D_GENGLOBALBYMST_H
#define I3D_GENGLOBALBYMST_H

//Mst minimal spanning tree 最小生成树算法
#include "i3d.h"
#include "Frame.h"
#include <vector>

int genGlobalByMst(const std::vector<i3d::Edge>& edges, std::vector<i3d::Frame>& kframes, std::vector<i3d::Edge>& kedges);

#endif //I3D_GENGLOBALBYMST_H
