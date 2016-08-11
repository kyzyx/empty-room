#include "rendering/opengl_compat.h"
#include "invrenderapp.h"
#include "rendering/rendermanager.h"
#include "rendering/hemicuberenderer.h"
#include "lighting/light.h"
#include "datamanager/imageio.h"
#include <pcl/console/parse.h>

using namespace std;

class WindowApp : public InvrenderApp {
    public:
        WindowApp() : res(500), generatemasks(false) {}
        virtual int run() {
            mmgr->loadSamples();
            RenderManager* rm = new RenderManager(mmgr);
            rm->setupMeshColors();
            int w = imgr->getCamera(0)->width;
            int h = imgr->getCamera(0)->height;
            float* im = new float[w*h*3];
            float* mask = new float[w*h*3];
            float* envmap = new float[res*res*6];
            float* wt = new float[res*res*2];
            memset(envmap, 0, sizeof(float)*res*res*6);
            memset(wt, 0, sizeof(float)*res*res*2);

            for (int z = 0; z < imgr->size(); z++) {
                const float* color = (const float*) imgr->getImage(z);
                const float* conf = (const float*) imgr->getImage("confidence", z);
                const CameraParams* cam = imgr->getCamera(z);
                rm->readFromRender(cam, im, VIEW_LABELS, true);
                for (int i = 0; i < h; i++) {
                    for (int j = 0; j < w; j++) {
                        bool isWindow = false;
                        int lightinfo = ftoi(im[3*(i*w+j) + 1]);
                        int lighttype = LIGHTTYPE(lightinfo);
                        if (lighttype & LIGHTTYPE_ENVMAP || lighttype & LIGHTTYPE_SH) {
                            isWindow = true;
                            R3Vector v;
                            v = cam->towards*cam->focal_length + (j-w/2+0.5)*cam->right + (i-h/2+0.5)*cam->up;
                            v.Normalize();
                            v = -v;
                            double phi = atan2(v.Y(), v.X());
                            double theta = acos(v.Z());
                            int ni = phi*res/M_PI + res;
                            int nj = theta*res/M_PI;
                            int idx = ni + 2*res*nj;
                            wt[idx] += conf[i*w+j];
                            for (int k = 0; k < 3; k++) {
                                envmap[3*idx+k] += conf[i*w+j]*color[3*(i*w+j)+k];
                            }
                        }
                        if (generatemasks) {
                            for (int k = 0; k < 3; k++) {
                                mask[3*(i*w+j)+k] = isWindow?1:0;
                            }
                        }
                    }
                }
                getProgressFunction(0,1)(100*z/imgr->size());
            }
            for (int i = 0; i < res*res*2; i++) {
                for (int k = 0; k < 3; k++) {
                    if (wt[i] > 0) envmap[3*i+k] /= wt[i];
                }
            }
            ImageIO::writeExrImage(filename, envmap, res*2, res);
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (!imgr) return 0;
            if (pcl::console::find_switch(argc, argv, "-generatemasks")) {
                generatemasks = true;
            }
            if (pcl::console::find_argument(argc, argv, "-filename") >= 0) {
                pcl::console::parse_argument(argc, argv, "-filename", filename);
            }
            if (pcl::console::find_argument(argc, argv, "-resolution") >= 0) {
                pcl::console::parse_argument(argc, argv, "-resolution", res);
            }
        }
        int res;
        string filename;
        bool generatemasks;
};

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1,1);
    glutCreateWindow("");
    glutHideWindow();
    WindowApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
