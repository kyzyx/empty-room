#ifndef _WALL_FINDER_H
#define _WALL_FINDER_H

#include <pcl/point_types.h>
#include "orientation_finder.h"

#include <vector>

class Segment {
    public:
    Segment(int d, int s, int e, int c) : direction(d), start(s), end(e), coord(c) {;}
    Segment() {;}
    std::pair<int,int> getCoords(int s) {
        if (direction) return std::make_pair(s, coord);
        else           return std::make_pair(coord, s);
    }
    int direction;
    int start;
    int end;
    int norm;
    int coord;
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
        WallFinder(double gridsize=0.01) : resolution(gridsize){;}
        /**
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         * Returns the floor plane level
         */
        double findFloorAndCeiling(
                OrientationFinder& of,
                std::vector<int>& labels,
                double anglethreshold=M_PI/40);
        /**
         * Label the point cloud with wall points
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         */
        void findWalls(
                OrientationFinder& of,
                std::vector<int>& labels,
                int wallthreshold=200,
                double minlength=0.2,
                double anglethreshold=M_PI/40);

        void loadWalls(std::string filename, std::vector<int>& labels);
        void saveWalls(std::string filename, std::vector<int>& labels);
        double getResolution() const { return resolution; }

        std::vector<Segment> wallsegments;
    private:
        double resolution;
        double findExtremal(
            pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
            Eigen::Vector3f dir,
            double anglethreshold,
            pcl::PointCloud<pcl::PointNormal>::Ptr outcloud);
};
#endif
