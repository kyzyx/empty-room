#ifndef _DISPLAY_H
#define _DISPLAY_H
#include "mesh.h"
#include "colorhelper.h"
#include "solver.h"
#include "wall_finder.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

enum {
    LABEL_NONE,
    LABEL_REPROJECT_DEBUG,
    LABEL_LIGHTS,
};

void visualize(Mesh& m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        ColorHelper& loader, InverseRender& ir, WallFinder& wf,
        int labeltype, int cameraid);
#endif
