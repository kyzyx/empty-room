#ifndef _FINDPLANEs_H
#define _FINDPLANES_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
/**
 * Locates the candidate wall/ceiling/floor planes in the point cloud
 * Each candidate plane (Ax + By + Cz + D = 0) is encoded as a Vector4 and
 * stored in the planes vector; each point in cloud will be labelled with the
 * corresponding index into planes, or -1 if it is not on a plane
 */
void findPlanes(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        std::vector<Eigen::Vector4d>& planes, std::vector<int>& ids);

#endif
