#ifndef _REPROJECT_H
#define _REPROJECT_H
#include "mesh.h"
#include "imagemanager.h"

void reproject(ImageManager& ch, ImageManager& lights, Mesh& mesh);
void reproject(
        ImageManager& hdr,
        Mesh& mesh,
        double threshold,
        bool flip_x=false,
        bool flip_y=false
);
void reproject(
        const float* hdrimage,
        const float* confidencemap,
        const float* depthmap,
        const CameraParams* cam,
        Mesh& mesh,
        double threshold,
        bool flip_x=false,
        bool flip_y=false
);
void reproject(const char* color, const char* light, const CameraParams* cam, Mesh& mesh);
#endif
