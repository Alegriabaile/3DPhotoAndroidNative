//
// Created by ale on 18-12-19.
//

#ifndef I3D_GENINITGRAPH_H
#define I3D_GENINITGRAPH_H

#include<vector>
#include "i3d.h"
#include "Frame.h"

int genInitGraph(const std::vector<i3d::Frame>& frames, const i3d::Intrinsics& intrinsics, std::vector<i3d::Edge>& edges);

#endif //I3D_GENINITGRAPH_H
