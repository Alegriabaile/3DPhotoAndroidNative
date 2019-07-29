//
// Created by ale on 19-2-17.
//

#ifndef I3D_WARPTOPANO_H
#define I3D_WARPTOPANO_H

#include "i3d.h"
#include "Frame.h"

void warpToPanos(std::vector<i3d::Frame>& kframes, i3d::Intrinsics& intrinsics);

#endif //I3D_WARPTOPANO_H
