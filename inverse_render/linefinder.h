#ifndef _LINEFINDER_H
#define _LINEFINDER_H
#include "datamanager/imagemanager.h"
#include "wallfinder/wall_finder.h"

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
        WallFinder& wf,
        double resolution,
        R4Matrix norm,
        std::vector<std::vector<Eigen::Vector3d> >& votes
);
void findWallLines(ImageManager& imgr, WallFinder& wf, std::vector<WallLine>& lines, double resolution=0.02, bool getvotes=false);

Eigen::Vector3d projectOntoWall(
        double x, double y,
        const CameraParams& cam,
        double ceilingplane, double floorplane,
        R4Matrix normalization,
        Segment& s);
#endif
