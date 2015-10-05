#include "invrenderapp.h"
#include "wallfinder/wall_finder.h"

#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;

class WallfindApp : public InvrenderApp {
    public:
        WallfindApp()
            : resolution(0.01), minlength(0.2),
              wallthreshold(200), anglethreshold(M_PI/40.),
              outputroommodel(""), flip(false)
        {;}

        virtual int run() {
            vector<char> types(mmgr->NVertices());
            PlaneOrientationFinder of(mmgr, resolution/2);
            of.computeNormals();
            progressfn(100,1,22);
            of.computeOrientation();
            progressfn(100,12,22);
            of.normalize();
            progressfn(100,13,22);
            WallFinder wf(resolution);
            Eigen::Matrix4f m = of.getNormalizationTransform();
            wf.setNormalizationTransform(m);

            wf.findFloorAndCeiling(&of, types, anglethreshold);
            progressfn(100,14,22);
            wf.findWalls(&of, types, wallthreshold, minlength, anglethreshold);
            {
                boost::interprocess::scoped_lock<MeshManager::shmutex> lock(*(mmgr->getMutex(MeshManager::TYPE_CHANNEL,0)));
                for (int i = 0; i < types.size(); ++i) {
                    mmgr->setLabel(i, types[i], MeshManager::TYPE_CHANNEL);
                }
            }
            roommodel::RoomModel r;
            wf.getAsRoomModel(&r);
            if (flip) {
                std::reverse(r.walls.begin(), r.walls.end());
            }

            Eigen::Matrix4f trans;
            trans.setIdentity();
            trans.block<3,1>(0,3) = -wf.getNormalizedWallEndpoint(0,true);
            m = trans*m;
            R4Matrix xform(
                m(0,0), m(0,1), m(0,2), m(0,3),
                m(1,0), m(1,1), m(1,2), m(1,3),
                m(2,0), m(2,1), m(2,2), m(2,3),
                m(3,0), m(3,1), m(3,2), m(3,3)
            );
            r.globaltransform = xform;
            if (!outputroommodel.empty()) {
                roommodel::save(r, outputroommodel);
            }
            emitDone();
            return 0;
        }
    protected:
        virtual void printargs() {
            cout <<
             "      -outputroommodel roomodel.json: output roommodel to file\n" \
             "      -anglethreshold f: Angle between normals to be\n" \
             "           considered equal (default PI/40)\n" \
             "      -flip: Reverse floor and ceiling\n" \
             "      -min_wall_length f: Minimum length of a wall\n" \
             "           (default 0.2)\n" \
             "      -resolution f: maximum distance for a point to\n" \
             "           be considered on a plane (default 0.01)\n" \
             "      -wallthreshold n: Minimum bucket count in histogram\n" \
             "           to count as a wall; dependent on resolution!\n" \
             "           (default 200)\n" << endl;
        }
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (pcl::console::find_switch(argc, argv, "-flip")) flip = true;
            if (pcl::console::find_argument(argc, argv, "-outputroommodel") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputroommodel", outputroommodel);
            }
            if (pcl::console::find_argument(argc, argv, "-anglethreshold") >= 0) {
                pcl::console::parse_argument(argc, argv, "-anglethreshold", anglethreshold);
            }
            if (pcl::console::find_argument(argc, argv, "-min_wall_length") >= 0) {
                pcl::console::parse_argument(argc, argv, "-min_wall_length", minlength);
            }
            if (pcl::console::find_argument(argc, argv, "-resolution") >= 0) {
                pcl::console::parse_argument(argc, argv, "-resolution", resolution);
            }
            if (pcl::console::find_argument(argc, argv, "-wallthreshold") >= 0) {
                pcl::console::parse_argument(argc, argv, "-wallthreshold", wallthreshold);
            }
            return 1;
        }

        string outputroommodel;
        double anglethreshold;
        double resolution;
        double minlength;
        int wallthreshold;
        bool flip;
};

int main(int argc, char** argv) {
    WallfindApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
