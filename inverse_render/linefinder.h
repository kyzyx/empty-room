#ifndef _LINEFINDER_H
#define _LINEFINDER_H
#include "colorhelper.h"
#include "wall_finder.h"

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

void findLines(const char* image, int w, int h, std::vector<Eigen::Vector4d>& lines);
void findWallLinesInImage(
        const CameraParams& cam,
        char* image,
        WallFinder& wf,
        double resolution,
        R4Matrix norm,
        std::vector<std::vector<double> >& votes
);
void findWallLines(ColorHelper& ch, WallFinder& wf, std::vector<WallLine>& lines, double resolution=0.02, bool getvotes=false);

#endif
