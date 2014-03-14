#ifndef _DISPLAY_H
#define _DISPLAY_H
#include "mesh.h"
#include "colorhelper.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

enum {
    LABEL_NONE,
    LABEL_REPROJECT_DEBUG,
    LABEL_LIGHTS,
};

void visualize(Mesh& m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        ColorHelper& loader,
        bool show_frustrum, bool prune, bool all_cameras,
        int labeltype, int cameraid);
#endif
