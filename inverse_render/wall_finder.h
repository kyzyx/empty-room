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
                std::vector<int>& labels,
                double anglethreshold=M_PI/40);
        /**
         * Label the point cloud with wall points
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         */
        void findWalls(
                std::vector<int>& labels,
                int wallthreshold=200,
                double minlength=0.2,
                double anglethreshold=M_PI/40);

        void loadWalls(std::string filename, std::vector<int>& labels);
        void saveWalls(std::string filename, std::vector<int>& labels);
        double getResolution() const { return resolution; }
        Eigen::Vector3f getWallEndpoint(int i, bool lo, double height=0) const;

        double floorplane;
        double ceilplane;
        std::vector<Segment> wallsegments;
    private:
        double resolution;
        OrientationFinder* of;
        double findExtremal(
            pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
            Eigen::Vector3f dir,
            double anglethreshold,
            pcl::PointCloud<pcl::PointNormal>::Ptr outcloud);
};
#endif
