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
            WallFinder wf;
            wf.loadFromRoomModel(room);
            vector<vector<Vector3d> > votes(wf.wallsegments.size());
            Matrix4f m4dn = wf.getNormalizationTransform();
            R4Matrix norm(
                    m4dn(0,0), m4dn(0,1), m4dn(0,2), m4dn(0,3),
                    m4dn(1,0), m4dn(1,1), m4dn(1,2), m4dn(1,3),
                    m4dn(2,0), m4dn(2,1), m4dn(2,2), m4dn(2,3),
                    m4dn(3,0), m4dn(3,1), m4dn(3,2), m4dn(3,3)
                    );
            for (int i = 0; i < imgr->size(); ++i) {
                findWallLinesInImage(*imgr, i, wf, 0.03, norm, votes);
                getProgressFunction(i,imgr->size())(100);
            }
            for (int i = 0; i < votes.size(); ++i) {
                for (int j = 0; j < votes[i].size(); ++j) {
                    WallLine wl(i, votes[i][j][0]);
                    wl.starty = votes[i][j][1];
                    wl.endy = votes[i][j][2];
                    printf(">>data:%d %f %f %f\n", wl.wallindex, wl.starty, wl.endy, wl.p);
                }
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
