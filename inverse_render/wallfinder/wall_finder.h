#ifndef _WALL_FINDER_H
#define _WALL_FINDER_H

#include "orientation_finder.h"
#include "roommodel/floorplanhelper.h"

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
        WallFinder(double gridsize=0.01)
            : resolution(gridsize){;}
        /**
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         * Returns the floor plane level
         */
        double findFloorAndCeiling(
                OrientationFinder* of,
                std::vector<char>& labels,
                double anglethreshold=M_PI/40);
        /**
         * Label the point cloud with wall points
         * resolution denotes the threshold beyond which points are not
         * considered on the same plane
         *
         */
        bool findWalls(
                OrientationFinder* of,
                std::vector<char>& labels,
                int wallthreshold=200,
                double minlength=0.2,
                double anglethreshold=M_PI/40);

        void getAsRoomModel(roommodel::RoomModel* rm);
        void loadFromRoomModel(roommodel::RoomModel* rm);
        void loadWalls(std::string filename, std::vector<char>& labels, OrientationFinder& of);
        void saveWalls(std::string filename, std::vector<char>& labels, OrientationFinder& of);
        double getResolution() const { return resolution; }
        Eigen::Vector3f getWallEndpoint(int i, bool lo, double height=0) const;
        Eigen::Vector3f getNormalizedWallEndpoint(int i, bool lo, double height=0) const;
        Eigen::Matrix4f getNormalizationTransform() const { return fph.normalization; }
        void setNormalizationTransform(Eigen::Matrix4f t) { fph.normalization = t; }

        double floorplane;
        double ceilplane;
        std::vector<Segment> wallsegments;
        std::vector<bool> forwards;
    private:
        FloorplanHelper fph;
        double resolution;
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
