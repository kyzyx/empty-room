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
        bool computeOrientation(double anglethreshold=M_PI/40, int resolution=100);
        void normalize();

        // Accessors
        Eigen::Matrix4f getNormalizationTransform() const { return normalizationtransform; }
        Eigen::Vector3f getAxis(int n) const { return axes[n]; }
        pcl::PointCloud<pcl::PointNormal>::ConstPtr getCloud() const {
            return pcl::PointCloud<pcl::PointNormal>::ConstPtr(cloud);
        }
        friend class WallFinder;
    protected:
        void addNormToHistogram(double x, double y, double z, int resolution, double weight);
        virtual bool prepareComputeOrientation() { return true; }
        virtual void fillHistogram(int resolution) = 0;

        // Histogram counters
        std::vector<double> totalweight;
        std::vector<std::vector<Eigen::Vector3f> > histogram;
        std::vector<std::vector<double> > histogramweights;

        // Basic geometry info
        pcl::PolygonMesh::Ptr mesh;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
        std::vector<Eigen::Vector3f> facenormals;

        std::vector<Eigen::Vector3f> axes;
        Eigen::Matrix4f normalizationtransform;
    private:
        OrientationFinder() {;}
};

class NormalOrientationFinder : public OrientationFinder {
    public:
        NormalOrientationFinder(pcl::PolygonMesh::Ptr m) : OrientationFinder(m) {;}
    protected:
        virtual bool prepareComputeOrientation();
        virtual void fillHistogram(int resolution);
};
class PlaneOrientationFinder : public OrientationFinder {
    public:
        PlaneOrientationFinder(pcl::PolygonMesh::Ptr m)
            : OrientationFinder(m), planeDistance(0.01) {;}
        PlaneOrientationFinder(pcl::PolygonMesh::Ptr m, double distanceThreshold)
            : OrientationFinder(m), planeDistance(distanceThreshold) {;}
    protected:
        virtual void fillHistogram(int resolution);
    private:
        double planeDistance;
};
#endif
