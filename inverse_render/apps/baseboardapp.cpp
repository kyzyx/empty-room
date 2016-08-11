#include "invrenderapp.h"
#include "findbaseboard.h"
#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;

class BaseboardApp : public InvrenderApp {
    public:
        BaseboardApp() {;}

        virtual int run() {
            BaseboardFinder bf(imgr, room);
            bf.compute();
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
    BaseboardApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
