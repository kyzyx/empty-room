#include "invrenderapp.h"
#include "orientededgefilter.h"
#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;

class EdgeImageApp : public InvrenderApp {
    public:
        EdgeImageApp() {;}

        virtual int run() {
            for (int i = 0; i < imgr->size(); ++i) {
                if (room) {
                    createEdgeImage(imgr->getCamera(i), room->globaltransform, imgr->getImage(i), imgr->getImageWriteable("edges",i));
                } else {
                    createEdgeImage(imgr->getCamera(i), imgr->getImage(i), imgr->getImageWriteable("edges",i));
                }
                int f = imgr->getFlags("edges", i);
                imgr->setFlags("edges", i, f|ImageManager::DF_INITIALIZED);
                if (noshm) imgr->saveImage("edges", i);
                getProgressFunction(i,imgr->size())(100);
            }
            emitDone();
            return 0;
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!imgr) return 0;
            return 1;
        }
};

int main(int argc, char** argv) {
    EdgeImageApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
