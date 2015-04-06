#ifndef _WALL_FINDER_H
#define _WALL_FINDER_H

#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include "orientation_finder.h"

#include <vector>

class GridSegment {
    public:
    GridSegment(int d, int s, int e, int c) : direction(d), start(s), end(e), coord(c) {;}
    GridSegment() {;}
    std::pair<int,int> getCoords(int s) const {
        if (direction) return std::make_pair(s, coord);
        else           return std::make_pair(coord, s);
    }
    int direction;
    int start;
    int end;
    int norm;
    int coord;
};
class Segment {
    public:
    Segment(const GridSegment& g, double resolution) {
        direction = g.direction;
        norm = g.norm;
        start = g.start*resolution;
        end = g.end*resolution;
        coord = g.coord*resolution;
    }
    Segment() {;}
    std::pair<double,double> getCoords(double s) const {
        if (direction) return std::make_pair(s, coord);
        else           return std::make_pair(coord, s);
    }
    bool pointOnSegment(Eigen::Vector3f p, double threshold) {
        // Check if on plane
        double c = direction?p(2):p(0);
        if (coord + threshold < c || coord - threshold > c) return false;
        // Check if within bounds
        c = direction?p(0):p(2);
        if (c > threshold + end || c < start - threshold) return false;
        return true;
    }
    double length() const { return end-start; }
    bool pointOnSegment(Eigen::Vector3f p, Eigen::Vector3f n, double threshold) {
        if (!pointOnSegment(p, threshold)) return false;
        Eigen::Vector3f axis = direction?Eigen::Vector3f::UnitZ():Eigen::Vector3f::UnitX();
        axis *= -norm;
        return n.dot(axis) > cos(M_PI/4);
    }

    int direction;
    double start;
    double end;
    int norm;
    double coord;
};
/**
 */
class WallFinder {
    public:
        enum Label {
            LABEL_NONE=0,
            LABEL_WALL,
            LABEL_CEILING,
            LABEL_FLOOR,
            LABEL_CORNER
        };
        WallFinder(OrientationFinder* orfinder, double gridsize=0.01)
            : of(orfinder), resolution(gridsize){;}
        WallFinder(pcl::PolygonMesh::Ptr mesh, double gridsize=0.01)
            : of(new PlaneOrientationFinder(mesh)), resolution(gridsize) {
                of->computeNormals();
                of->computeOrientation();
                of->normalize();
            }
        /**
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         * Returns the floor plane level
         */
        double findFloorAndCeiling(
                std::vector<char>& labels,
                double anglethreshold=M_PI/40);
        /**
         * Label the point cloud with wall points
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         */
        void findWalls(
                std::vector<char>& labels,
                int wallthreshold=200,
                double minlength=0.2,
                double anglethreshold=M_PI/40);

        void loadWalls(std::string filename, std::vector<char>& labels);
        void saveWalls(std::string filename, std::vector<char>& labels);
        double getResolution() const { return resolution; }
        Eigen::Vector3f getWallEndpoint(int i, bool lo, double height=0) const;
        Eigen::Vector3f getNormalizedWallEndpoint(int i, bool lo, double height=0) const;
        Eigen::Matrix4f getNormalizationTransform() const { return of->getNormalizationTransform(); }

        double floorplane;
        double ceilplane;
        std::vector<Segment> wallsegments;
        std::vector<bool> forwards;
    private:
        double resolution;
        OrientationFinder* of;
        double findExtremalHistogram(
            pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
            Eigen::Vector3f dir,
            double resolution, double threshold);
        double findExtremalNormal(
            pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
            Eigen::Vector3f dir,
            double anglethreshold,
            pcl::PointCloud<pcl::PointNormal>::Ptr outcloud);
};
#endif
