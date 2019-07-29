//
// Created by ale on 18-12-18.
//

#ifndef I3D_INITINPUTDATA_H
#define I3D_INITINPUTDATA_H

#include <vector>
#include <string>
#include "i3d.h"
#include "Frame.h"
#include "readData.h"

namespace i3d
{
    int initFrames(const std::string & mainDir, std::vector<Frame>& frames, i3d::Intrinsics& intrinsic);
}


#endif //I3D_INITFRAMES_H
