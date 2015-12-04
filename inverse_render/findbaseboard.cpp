#include "findbaseboard.h"
#include "linefinder.h"

using namespace std;

void BaseboardFinder::compute(float resolution, float edgethreshold) {
    WallFinder wf;
    wf.loadFromRoomModel(floorplan);
    vector<vector<vector<float> > > hist(floorplan->height/resolution+4); // [height][wallidx][pos]
    for (int i = 0; i < hist.size(); i++) {
        hist[i].resize(wf.wallsegments.size());
        for (int j = 0; j < wf.wallsegments.size(); j++) {
            hist[i][j].resize(wf.wallsegments[j].length()/resolution+2,0);
        }
    }
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
                    cout << x << " " << y << " ";
                    Eigen::Vector3d p = projectOntoFloorplan(x, y, *cam, wf);
                    if (p[0] >= 0) {
                        int h = (p[2] - wf.floorplane)/resolution;
                        if (h >= hist.size()) h = hist.size()-1;

                        int xx = p[1]/resolution;
                        if (xx < 0) xx = 0;
                        else if (xx >= hist[h][p[0]].size()) xx = hist[h][p[0]].size()-1;

                        float v = 0;
                        if (wf.wallsegments[p[0]].direction == 0 && !isinf(img[idx]))
                            v = img[idx];
                        if (wf.wallsegments[p[0]].direction == 1 && !isinf(img[idx+2]))
                            v = img[idx+2];
                        cout << h << " " << p[0] << " " << xx;
                        hist[h][p[0]][xx] = max(hist[h][p[0]][xx], log(v+1));
                    }
                    cout << endl;
                }
            }
        }
        cout << "Done " << i << "/" << imgr->size() << endl;
        cin.get();
    }

    double best = 0;
    for (int i = 0; i < hist.size(); i++) {
        double s = 0;
        for (int j = 0; j < hist[i].size(); j++) {
            for (int k = 0; k < hist[i][j].size(); k++) {
                s += hist[i][j][k];
            }
        }
        printf("%d (%.4f): %f\n", i, i*resolution, s);
        if (s > best) {
            best = s;
            bbh = i*resolution;
        }
    }
    cout << "Best: " << best << " " << bbh << endl;
    bbd = 0;
    bbcolor = roommodel::Color(0.5,0.4,0.3);
    solved = true;
}
