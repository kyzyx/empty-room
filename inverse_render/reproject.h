#ifndef _REPROJECT_H
#define _REPROJECT_H
#include "mesh.h"
#include "colorhelper.h"

void reproject(ColorHelper& ch, ColorHelper& lights, Mesh& mesh);
void reproject(
        ColorHelper& hdr,
        Mesh& mesh,
        double threshold,
        bool flip_x=false,
        bool flip_y=false
);
void reproject(
        const float* hdrimage,
        const CameraParams* cam,
        Mesh& mesh,
        double threshold,
        bool flip_x=false,
        bool flip_y=false
);
void reproject(const char* color, const char* light, const CameraParams* cam, Mesh& mesh);
#endif
