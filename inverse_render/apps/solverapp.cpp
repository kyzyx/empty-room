#include "rendering/opengl_compat.h"
#include "invrenderapp.h"
#include "wallfinder/wall_finder.h"
#include "solver.h"
#include "rerender.h"
#include <iostream>
#include <pcl/console/parse.h>

using namespace std;

class SolverApp : public InvrenderApp {
    public:
        SolverApp() : hemicuberesolution(150), discardthreshold(0.25), matlabfilename(""), label(WallFinder::LABEL_WALL), cameranum(0), scale(0) {}
        virtual int run() {
            mmgr->loadSamples();
            InverseRender ir(mmgr, hemicuberesolution, getProgressFunction(1,2));
            vector<SampleData> walldata;
            vector<int> wallindices;
            for (int i = 0; i < mmgr->size(); i++) {
                if (mmgr->getLabel(i, 1) == label)
                    wallindices.push_back(i);
            }
            ir.computeSamples(walldata, wallindices, numsamples, discardthreshold, false, getProgressFunction(0,2));
            if (matlabfilename.length())
                ir.writeVariablesMatlab(walldata, matlabfilename);
            if (scale > 0) {
                ir.setLossFunction(LOSS_HUBER);
                ir.setLossFunctionScale(scale);
            }
            ir.solve(walldata);
            cout << "data:WallMaterial " << ir.wallMaterial.r << " " << ir.wallMaterial.g << " " << ir.wallMaterial.b << endl;
            for (int i = 0; i < ir.lights.size(); i++) {
                cout << "data:Light " << i << " " << ir.lights[i].r << " " << ir.lights[i].g << " " << ir.lights[i].b << endl;
            }
            if (pbrtfilename.length()) {
                room->wallMaterial.diffuse.r = ir.wallMaterial.r;
                room->wallMaterial.diffuse.g = ir.wallMaterial.g;
                room->wallMaterial.diffuse.b = ir.wallMaterial.b;
                if (imgr && room) {
                    outputPbrtFile(
                            pbrtfilename, room, *mmgr, ir.lights,
                            imgr->getCamera(cameranum));
                } else {
                    cout << "Error - need to supply imagemanager and roommodel to output pbrt file" << endl;
                }
            }
        }
    protected:
        virtual void printargs() {

        }
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (pcl::console::find_argument(argc, argv, "-hemicuberesolution") >= 0) {
                pcl::console::parse_argument(argc, argv, "-hemicuberesolution", hemicuberesolution);
            }
            if (pcl::console::find_argument(argc, argv, "-label") >= 0) {
                pcl::console::parse_argument(argc, argv, "-label", label);
            }
            if (pcl::console::find_argument(argc, argv, "-discardthreshold") >= 0) {
                pcl::console::parse_argument(argc, argv, "-discardthreshold", discardthreshold);
            }
            if (pcl::console::find_argument(argc, argv, "-numsamples") >= 0) {
                pcl::console::parse_argument(argc, argv, "-numsamples", numsamples);
            }
            if (pcl::console::find_argument(argc, argv, "-scale") >= 0) {
                pcl::console::parse_argument(argc, argv, "-scale", scale);
            }
            if (pcl::console::find_argument(argc, argv, "-outputsamplesfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputsamplesfile", matlabfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputpbrtfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputpbrtfile", pbrtfilename);
            }
            return true;
        }
        int numsamples;
        float discardthreshold;
        float scale;
        int hemicuberesolution;
        string matlabfilename;
        string pbrtfilename;
        int cameranum;
        int label;
};

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1,1);
    glutCreateWindow("");
    glutHideWindow();
    SolverApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
