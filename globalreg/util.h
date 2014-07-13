#ifndef _UTIL_H
#define _UTIL_H
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

const double EPSILON = 0.00001;

double dist2(pcl::PointXYZ a, pcl::PointXYZ b);
bool isValid(pcl::PointXYZ p);

inline Eigen::Vector3d getNormal(pcl::PointNormal p) {
    return Eigen::Vector3d(p.normal_x, p.normal_y, p.normal_z);
}
inline double safe_acos(double theta) {
    if (theta < 1 - EPSILON && theta > EPSILON - 1) return acos(theta);
    else if (theta > 0) return 0;
    else return M_PI;
}

/**
 * Returns the centroid of the cloud
 */
Eigen::Vector3d cloudMidpoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
/**
 * Returns the centroid of the union of two clouds
 */
Eigen::Vector3d cloudMidpoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);

/**
 * Applies the transform to the plane in Ax + By + Cz + D = 0 form. Assumes the
 * normal (A,B,C) is normalized.
 */
Eigen::Vector4d transformPlane(Eigen::Vector4d plane, Eigen::Matrix4d transform);
#endif
