#include "invrenderapp.h"
#include "reproject.h"
#include "clusterlights.h"
#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;

class ReprojectApp : public InvrenderApp {
    public:
        ReprojectApp() : projectonly(-1), hdr_threshold(10), minlightsize(100), dosamples(true) {;}

        virtual int run() {
            if (dosamples) {
                if (projectonly < 0) {
                    reproject(*imgr, *mmgr, hdr_threshold, boost::bind(&ReprojectApp::refreshingProgressFn, this, _1));
                } else {
                    reproject((const float*) imgr->getImage(projectonly),
                              (const float*) imgr->getImage("confidence", projectonly),
                              (const float*) imgr->getImage("depth", projectonly),
                              imgr->getCamera(projectonly),
                              *mmgr, hdr_threshold);
                }
                mmgr->commitSamples();
            } else {
                mmgr->loadSamples();
            }
            int numlights = clusterLights(*mmgr, hdr_threshold, minlightsize);
            emitDone();
            return 0;
        }
    protected:
        void refreshingProgressFn(int percent) {
            mmgr->commitSamples(false);
            getProgressFunction(0,1)(percent);
        }
        virtual void printargs() {
cout << "       -project n: Reproject only image n\n" << endl;
cout << "       -hdr_threshold f: Set intensity threshold for lights as f\n" << endl;
cout << "       -min_light_size n: Minimum number of vertices in a light\n" << endl;
cout << "       -label_lights_only: Don't reproject, just label lights\n" << endl;
        }
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr || !imgr) return 0;
            if (pcl::console::find_switch(argc, argv, "-label_lights_only")) dosamples = false;
            if (pcl::console::find_argument(argc, argv, "-project") >= 0) {
                pcl::console::parse_argument(argc, argv, "-project", projectonly);
            }
            if (pcl::console::find_argument(argc, argv, "-hdr_threshold") >= 0) {
                pcl::console::parse_argument(argc, argv, "-hdr_threshold", hdr_threshold);
            }
            if (pcl::console::find_argument(argc, argv, "-min_light_size") >= 0) {
                pcl::console::parse_argument(argc, argv, "-min_light_size", minlightsize);
            }
            return 1;
        }

        int projectonly;
        double hdr_threshold;
        int minlightsize;
        bool dosamples;
};

int main(int argc, char** argv) {
    ReprojectApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
