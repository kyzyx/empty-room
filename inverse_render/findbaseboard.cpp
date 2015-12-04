#include "findbaseboard.h"
#include "linefinder.h"

using namespace std;

void BaseboardFinder::compute(float resolution, float edgethreshold) {
    vector<float> hist(floorplan->height/resolution+4,0);
    WallFinder wf;
    wf.loadFromRoomModel(floorplan);
    for (int i = 0; i < imgr->size(); i++) {
        const float* img = (const float*) imgr->getImage("edges", i);
        const char* mask = (const char*) imgr->getImage("labels", i);
        const CameraParams* cam = imgr->getCamera(i);
        for (int y = 0; y < cam->height; y++) {
            for (int x = 0; x < cam->width; x++) {
                int idx = 3*(y*cam->width+x);
                if (mask[y*cam->width+x] != WallFinder::LABEL_WALL) continue;
                if ((!isinf(img[idx]) && img[idx] > edgethreshold) ||
                    (!isinf(img[idx+2]) && img[idx+2] > edgethreshold)) {
                    Eigen::Vector3d p = projectOntoFloorplan(x, y, *cam, wf);
                    if (p[0] >= 0) {
                        int id = (p[2] - wf.floorplane)/resolution;
                        if (id >= hist.size()) id = hist.size()-1;
                        float v = 0;
                        if (wf.wallsegments[p[0]].direction == 0 && !isinf(img[idx]))
                            v = img[idx];
                        if (wf.wallsegments[p[0]].direction == 1 && !isinf(img[idx+2]))
                            v = img[idx+2];
                        hist[id] += log(v+1);
                    }
                }
            }
        }
        cout << "Done " << i << "/" << imgr->size() << endl;
    }

    double best = 0;
    for (int i = 0; i < hist.size(); i++) {
        printf("%d (%.4f): %f\n", i, i*resolution, hist[i]);
        if (hist[i] > best) {
            best = hist[i];
            bbh = i*resolution;
        }
    }
    cout << "Best: " << best << " " << bbh << endl;
    bbd = 0;
    bbcolor = roommodel::Color(0.5,0.4,0.3);
    solved = true;
}
