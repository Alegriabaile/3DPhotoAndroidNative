//
// Created by ale on 19-4-18.
//

#ifndef I3D_GENRELATIVEPOSES_H
#define I3D_GENRELATIVEPOSES_H
#include<vector>
#include "i3d.h"
#include "Frame.h"

int genRelativePoses(const std::vector<i3d::Frame>& frames, const i3d::Intrinsics& intrinsics, std::vector<i3d::Edge>& edges);

#endif //I3D_GENRELATIVEPOSES_H
