#ifndef _LINEFINDER_H
#define _LINEFINDER_H
#include "datamanager/imagemanager.h"
#include <boost/function.hpp>
#include "roommodel/floorplanhelper.h"

class WallLine {
    public:
        WallLine(int wallidx, double pos)
            : wallindex(wallidx), vertical(true), p(pos) {;}
        int wallindex;
        bool vertical;
        double starty;
        double endy;
        double p;
};

void findWallLinesInImage(
        ImageManager& imgr,
        int idx,
        FloorplanHelper& floorplan,
        double resolution,
        std::vector<std::vector<Eigen::Vector3d> >& verticalvotes,
        std::vector<std::vector<Eigen::Vector3d> >& horizontalvotes
);
void findWallLines(
        ImageManager& imgr,
        FloorplanHelper& floorplan,
        std::vector<WallLine>& lines,
        double resolution=0.02,
        boost::function<void(int)> cb=NULL
);

// Projects point x,y (from top left corner) of the image taken from cam
// onto the wall defined by the remaining parameters.
// Returns v describing the projected point P
//   v[0] = distance to P from start of wall
//   v[1] = height from ground of P
//   v[2] = distance from P to camera, or infinity if P does not hit the wall
Eigen::Vector3d projectOntoWall(
        double x, double y,
        const CameraParams& cam,
        double ceilingplane, double floorplane,
        R4Matrix normalization,
        Segment& s);

// Projects point x,y (from top left corner) of the image taken from cam
// onto the floorplan given by wf
// Returns v describing the projected point P
//   v[0] = index of wall onto which P projects, or negative if it does
//          not hit a wall
//   v[1] = distance to P from start of wall
//   v[2] = height from ground of P
Eigen::Vector3d projectOntoFloorplan(
        double x, double y,
        const CameraParams& cam,
        FloorplanHelper& floorplan);
#endif
