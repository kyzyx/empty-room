#ifndef _ORIENTATION_FINDER_H
#define _ORIENTATION_FINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

/**
 * OrientationFinder encapsulates an algorithm to find the
 * dominant directions of a mesh to establish a coordinate
 * system based on the face normals.
 *
 * Coordinate system assumes y axis is up
 */
class OrientationFinder {
    public:
        OrientationFinder(pcl::PolygonMesh::Ptr m);

        bool computeNormals(bool ccw=false);
        /**
         * Computes the dominant three axes based on face normals.
         * resolution lists how many buckets per M_PI/4 radians
         * anglethreshold lists the absolute angle off of a right angle
         *      that axes can be at
         */
        bool computeOrientation(int resolution=100, double anglethreshold=M_PI/40);

        // Accessors
        Eigen::Vector3f getAxis(int n) const { return axes[n]; }
        pcl::PointCloud<pcl::PointNormal>::ConstPtr getCloud() const {
            return pcl::PointCloud<pcl::PointNormal>::ConstPtr(cloud);
        }
    private:
        OrientationFinder() {;}

        // Basic geometry info
        pcl::PolygonMesh::Ptr mesh;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
        std::vector<Eigen::Vector3f> facenormals;

        std::vector<Eigen::Vector3f> axes;
};
#endif
