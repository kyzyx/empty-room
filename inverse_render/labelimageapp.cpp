#include "opengl_compat.h"
#include "invrenderapp.h"
#include "hemicuberenderer.h"
#include <string>
#include <iostream>
#include <pcl/console/parse.h>
#include <boost/bind.hpp>

using namespace std;

class LabelImageApp : public InvrenderApp {
    public:
        LabelImageApp() {;}

        virtual int run() {
            glewInit();
            RenderManager rm(mmgr);
            rm.setupMeshColors();
            for (int i = 0; i < imgr->size(); ++i) {
                rm.createLabelImage(imgr->getCamera(i), imgr->getImageWriteable("labels",i));
                int f = imgr->getFlags("labels", i);
                imgr->setFlags("labels", i, f|ImageManager::DF_INITIALIZED);
                getProgressFunction(i,imgr->size())(100);
            }
            emitDone();
            return 0;
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr || !imgr) return 0;
            return 1;
        }
};

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1,1);
    glutCreateWindow("");
    glutHideWindow();
    LabelImageApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
