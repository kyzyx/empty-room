#ifndef _FLOORPLANHELPER_H
#define _FLOORPLANHELPER_H

#include "roommodel/roommodel.h"
#include <Eigen/Eigen>
#include <vector>
#include <utility>

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
    bool isCompatiblePoint(Eigen::Vector3f p, Eigen::Vector3f n, double threshold) {
        Eigen::Vector3f axis = direction?Eigen::Vector3f::UnitZ():Eigen::Vector3f::UnitX();
        axis *= -norm;
        if (n.dot(axis) < cos(M_PI/4)) return false;
        // Check if on plane
        double c = direction?p(2):p(0);
        //if (coord + threshold < c || coord - threshold > c) return false;
        // Check if within bounds
        c = direction?p(0):p(2);
        if (c > threshold + end || c < start - threshold) return false;
        return true;
    }

    int direction;
    double start;
    double end;
    int norm;
    double coord;
};

class FloorplanHelper {
    public:
        FloorplanHelper() {}
        void getAsRoomModel(roommodel::RoomModel* rm);
        void loadFromRoomModel(roommodel::RoomModel* rm);

        Eigen::Vector3f getWallEndpoint(int i, bool lo, double height=0) const;
        Eigen::Vector3f getNormalizedWallEndpoint(int i, bool lo, double height=0) const;
        Eigen::Matrix4f getNormalizationTransform() const { return normalization; }
        void setNormalizationTransform(Eigen::Matrix4f t) { normalization = t; }

        std::vector<Segment> wallsegments;
        std::vector<bool> forwards;
        double floorplane;
        double ceilplane;
        Eigen::Matrix4f normalization;
};
#endif
