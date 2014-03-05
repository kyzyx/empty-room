#ifndef _ORIENTATION_FINDER_H
#define _ORIENTATION_FINDER_H

#include <pcl/point_types.h>
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
        bool computeOrientation();

        // Accessors
        Eigen::Vector3f getAxis(int n) const { return axes[n]; }
        pcl::PointCloud<pcl::PointNormal>::ConstPtr getCloud() const {
            return pcl::PointCloud<pcl::PointNormal>::ConstPtr(cloud);
        }

        int getResolution() const { return resolution; }
        void setResolution(int r) { resolution = r; }
    private:
        OrientationFinder();

        // Basic geometry info
        pcl::PolygonMesh::Ptr mesh;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
        std::vector<Eigen::Vector3f> facenormals;

        std::vector<Eigen::Vector3f> axes;

        // Parameters for axis finding
        int resolution; // Angle buckets per M_PI/4
        double anglethreshold; // Amount away from perpendicular to check for histogram peaks
};
#endif
