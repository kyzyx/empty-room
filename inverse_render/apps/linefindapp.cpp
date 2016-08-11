#include "invrenderapp.h"
#include "linefinder.h"
#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;

class LineFindApp : public InvrenderApp {
    public:
        LineFindApp() {;}

        virtual int run() {
            FloorplanHelper fp;
            fp.loadFromRoomModel(room);
            vector<WallLine> lines;
            findWallLines(*imgr, fp, lines, 0.03, getProgressFunction(0,1));
            for (int i = 0; i < lines.size(); i++) {
                Vector3f p1, p2;
                if (lines[i].vertical) {
                    p1 = fp.getWallPoint(lines[i].wallindex, lines[i].p, lines[i].starty);
                    p2 = fp.getWallPoint(lines[i].wallindex, lines[i].p, lines[i].endy);
                } else {
                    p1 = fp.getWallPoint(lines[i].wallindex, lines[i].starty, lines[i].p);
                    p2 = fp.getWallPoint(lines[i].wallindex, lines[i].endy, lines[i].p);
                }

                printf(">>data:%d %f %f %f %f %f %f %f %f %f\n", lines[i].wallindex, lines[i].starty, lines[i].endy, lines[i].p, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
            }
            emitDone();
            return 0;
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!imgr) return 0;
            if (!room) return 0;
            return 1;
        }
};

int main(int argc, char** argv) {
    LineFindApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
