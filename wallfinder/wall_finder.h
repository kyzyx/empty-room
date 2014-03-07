#ifndef _WALL_FINDER_H
#define _WALL_FINDER_H

#include <pcl/point_types.h>
#include "orientation_finder.h"

#include <vector>

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
        WallFinder() {;}
        /**
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         * Returns the floor plane level
         */
        double findFloorAndCeiling(
                OrientationFinder& of,
                std::vector<int>& labels,
                double resolution=0.01);
        /**
         * Label the point cloud with wall points
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         */
        void findWalls(
                OrientationFinder& of,
                std::vector<int>& labels,
                double minlength=0.2,
                double resolution=0.01);

    private:
        double findExtremal(
            pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
            Eigen::Vector3f dir,
            double anglethreshold,
            double resolution,
            pcl::PointCloud<pcl::PointNormal>::Ptr outcloud);
};
#endif
