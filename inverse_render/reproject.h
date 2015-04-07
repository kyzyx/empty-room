#ifndef _REPROJECT_H
#define _REPROJECT_H
#include "meshmanager.h"
#include "imagemanager.h"

void reproject(
        ImageManager& hdr,
        MeshManager& mesh,
        double threshold,
        bool flip_x=false,
        bool flip_y=false
);
void reproject(
        const float* hdrimage,
        const float* confidencemap,
        const float* depthmap,
        const CameraParams* cam,
        MeshManager& mesh,
        double threshold,
        bool flip_x=false,
        bool flip_y=false,
        R3MeshSearchTree* searchtree=NULL
);
#endif
