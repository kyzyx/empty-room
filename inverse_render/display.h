#ifndef _DISPLAY_H
#define _DISPLAY_H
#include "mesh.h"
#include "colorhelper.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void visualize(Mesh& m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        ColorHelper& loader,
        bool show_frustrum, bool prune, bool all_cameras,
        bool project_debug, int cameraid);
#endif
