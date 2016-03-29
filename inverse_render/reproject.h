#ifndef _REPROJECT_H
#define _REPROJECT_H
#include "datamanager/meshmanager.h"
#include "datamanager/imagemanager.h"

void reproject(
        ImageManager& hdr,
        MeshManager& mesh,
        double threshold,
        boost::function<void(int)> cb=NULL
);
void reproject(
        const float* hdrimage,
        const float* confidencemap,
        const float* depthmap,
        const CameraParams* cam,
        MeshManager& mesh,
        double threshold,
        int16_t id=-1
);
#endif
