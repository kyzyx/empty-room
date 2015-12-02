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
                    double bestd = numeric_limits<double>::infinity();
                    double besth = 0;
                    int bestj = -1;
                    for (int j = 0; j < wf.wallsegments.size(); j++) {
                        Eigen::Vector3d p = projectOntoWall(
                                x, y, *cam,
                                wf.ceilplane, wf.floorplane,
                                floorplan->globaltransform,
                                wf.wallsegments[j]);
                        if (p[2] < bestd) {
                            bestd = p[2];
                            besth = p[1];
                            bestj = j;
                        }
                    }
                    if (bestd < numeric_limits<double>::infinity()) {
                        int id = (besth - wf.floorplane)/resolution;
                        if (id >= hist.size()) id = hist.size()-1;
                        float v = 0;
                        if (wf.wallsegments[bestj].direction == 0 && !isinf(img[idx]))
                            v = img[idx];
                        if (wf.wallsegments[bestj].direction == 1 && !isinf(img[idx+2]))
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
