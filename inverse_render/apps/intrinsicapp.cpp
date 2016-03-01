#include "rendering/opengl_compat.h"
#include "invrenderapp.h"
#include "wallfinder/wall_finder.h"
#include "solver.h"
#include "rerender.h"
#include <iostream>
#include <fstream>
#include <pcl/console/parse.h>

using namespace std;

class SolverApp : public InvrenderApp {
    public:
        SolverApp() : hemicuberesolution(150), scale(1), label(-1) {}
        virtual int run() {
            mmgr->loadSamples();
            RenderManager rm(mmgr);
            rm.setupMeshColors();
            rm.precalculateAverageSamples();
            HemicubeRenderer hr(&rm, hemicuberesolution);

            vector<Material> colors;
            float* i1 = new float[3*hemicuberesolution*hemicuberesolution];
            float* i2 = new float[3*hemicuberesolution*hemicuberesolution];
            int nt = 0;
            int it = 0;
            if (label < 0) {
                nt = mmgr->NVertices();
            }
            else {
                for (int i = 0; i < mmgr->size(); i++) {
                    if (mmgr->getLabel(i, 1) == label) nt++;
                }
            }
            int interval = nt/100;

            vector<Material> lights;
            if (useLights) {
                ifstream in;
                int nlights;
                in >> nlights;
                for (int i = 0; i < nlights; i++) {
                    double r, g, b;
                    in >> r >> g >> b;
                    lights.push_back(Material(r,g,b));
                }
            }

            for (int i = 0; i < mmgr->NVertices(); i++) {
                if (label >= 0 && mmgr->getLabel(i,1) != label) {
                    colors.push_back(Material(0,0,0));
                    continue;
                }
                if (i%interval == 0) progressfn(100, it, nt);
                it++;
                cout << it << "/" << nt << endl;
                SampleData s = hr.computeSample(i, i1, i2);
                for (int j = 0; j < s.lightamount.size(); j++) {
                    s.radiosity += lights[j]*s.lightamount[j];
                }
                s.netIncoming *= (1-s.fractionDirect)/(1-s.fractionDirect-s.fractionUnknown);
                float r = s.radiosity.r/s.netIncoming.r;
                float g = s.radiosity.g/s.netIncoming.g;
                float b = s.radiosity.b/s.netIncoming.b;
                colors.push_back(Material(r,g,b));
            }
            if (plyfilename.length()) {
                mmgr->writePlyMesh(plyfilename, colors, scale);
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
            if (pcl::console::find_argument(argc, argv, "-outputplyfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputplyfile", plyfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputscale") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputscale", scale);
            }
            if (pcl::console::find_argument(argc, argv, "-label") >= 0) {
                pcl::console::parse_argument(argc, argv, "-label", label);
            }
            if (pcl::console::find_argument(argc, argv, "-lightfile") >= 0) {
                pcl::console::parse_argument(argc, argv, "-lightfile", lightfilename);
                useLights = true;
            }
            return true;
        }
        int hemicuberesolution;
        double scale;
        int label;
        string plyfilename;
        string lightfilename;
        bool useLights;
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
