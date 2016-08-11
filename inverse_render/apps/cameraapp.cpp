#include "invrenderapp.h"
#include "rerender.h"
#include <iostream>
#include <pcl/console/parse.h>

using namespace std;
using namespace Eigen;

Matrix3d getRotationMatrix(const CameraParams* cam) {
    Matrix3d ret;
    ret << cam->right[0], cam->towards[0], cam->up[0],
           cam->right[1], cam->towards[1], cam->up[1],
           cam->right[2], cam->towards[2], cam->up[2];
    return ret;
}
CameraParams fromRotationMatrix(Eigen::Matrix3d m) {
    CameraParams ret;
    Vector3d right = m*Vector3d(1,0,0);
    Vector3d up = m*Vector3d(0,0,1);
    Vector3d towards = m*Vector3d(0,1,0);
    ret.up = R3Vector(up[0], up[1], up[2]);
    ret.right = R3Vector(right[0], right[1], right[2]);
    ret.towards = R3Vector(towards[0], towards[1], towards[2]);
    return ret;
}

class CameraApp : public InvrenderApp {
    public:
        CameraApp() : startindex(0), starti(0), endi(-1), interp(1), rotationangle(0) {}
        virtual int run() {
            int frameno = startindex;
            if (rotationangle != 0) {
                double a = 0;
                int i = starti;
                CameraParams cam;
                cam.pos = imgr->getCamera(i)->pos;
                cam.gamma = imgr->getCamera(i)->gamma;
                cam.width = imgr->getCamera(i)->width;
                cam.height = imgr->getCamera(i)->height;
                cam.focal_length = imgr->getCamera(i)->focal_length;
                cam.fov = imgr->getCamera(i)->fov;
                cam.up = imgr->getCamera(i)->up;
                cam.towards = imgr->getCamera(i)->towards;
                R3Vector v = imgr->getCamera(i+1)->pos - imgr->getCamera(i)->pos;
                v /= interp;
                char buf[100];
                for (int i = 0; i <= interp; i++) {
                    snprintf(buf, 100, "%s%04d.pbrt", pbrtfilename.c_str(), frameno++);
                    outputPbrtCameraFile(
                            buf, pbrtfilename+".pbrt",
                            &cam);
                    cam.towards.Rotate(cam.up, rotationangle/interp);
                    cam.pos += v;
                }
                return 0;
            }
            if (endi < 0) endi = imgr->size();
            for (int i = starti; i < endi; i++) {
                char buf[100];
                snprintf(buf, 100, "%s%04d.pbrt", pbrtfilename.c_str(), frameno++);
                outputPbrtCameraFile(
                        buf, pbrtfilename+".pbrt",
                        imgr->getCamera(i));
                if (i == starti) continue;
                for (int j = 1; j < interp; j++) {
                    CameraParams cam;
                    Eigen::Quaterniond q(getRotationMatrix(imgr->getCamera(i)));
                    Eigen::Quaterniond q2(getRotationMatrix(imgr->getCamera(i-1)));
                    Eigen::Quaterniond q3 = q2.slerp(j/(double)interp,q);
                    cam = fromRotationMatrix(q3.toRotationMatrix());
                    cam.pos = (imgr->getCamera(i)->pos + imgr->getCamera(i-1)->pos)/2;
                    cam.gamma = imgr->getCamera(i)->gamma;
                    cam.width = imgr->getCamera(i)->width;
                    cam.height = imgr->getCamera(i)->height;
                    cam.focal_length = imgr->getCamera(i)->focal_length;
                    cam.fov = imgr->getCamera(i)->fov;
                    snprintf(buf, 100, "%s%04d.pbrt", pbrtfilename.c_str(), frameno++);
                    outputPbrtCameraFile(
                            buf, pbrtfilename+".pbrt",
                            &cam);
                }
            }
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!imgr) return 0;
            if (pcl::console::find_argument(argc, argv, "-start") >= 0) {
                pcl::console::parse_argument(argc, argv, "-start", starti);
            }
            if (pcl::console::find_argument(argc, argv, "-end") >= 0) {
                pcl::console::parse_argument(argc, argv, "-end", endi);
            }
            if (pcl::console::find_argument(argc, argv, "-interpolation") >= 0) {
                pcl::console::parse_argument(argc, argv, "-interpolation", interp);
            }
            if (pcl::console::find_argument(argc, argv, "-filename") >= 0) {
                pcl::console::parse_argument(argc, argv, "-filename", pbrtfilename);
            }
            if (pcl::console::find_argument(argc, argv, "-outputindex") >= 0) {
                pcl::console::parse_argument(argc, argv, "-outputindex", startindex);
            }
            if (pcl::console::find_argument(argc, argv, "-rotate") >= 0) {
                pcl::console::parse_argument(argc, argv, "-rotate", rotationangle);
            }
            return 1;
        }
        int starti, endi, interp, startindex;
        double rotationangle;
        string pbrtfilename;
};
int main(int argc, char** argv) {
    CameraApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
