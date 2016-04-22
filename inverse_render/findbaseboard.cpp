#include "findbaseboard.h"
#include "linefinder.h"

#include "datamanager/imageio.h"
using namespace std;

void BaseboardFinder::compute(float resolution, float edgethreshold) {
    WallFinder wf;
    wf.loadFromRoomModel(floorplan);
    vector<vector<vector<float> > > hist(floorplan->height/resolution+4); // [height][wallidx][pos]
    vector<vector<vector<int> > > count(floorplan->height/resolution+4); // [height][wallidx][pos]
    for (int i = 0; i < hist.size(); i++) {
        hist[i].resize(wf.wallsegments.size());
        count[i].resize(wf.wallsegments.size());
        for (int j = 0; j < wf.wallsegments.size(); j++) {
            hist[i][j].resize(wf.wallsegments[j].length()/resolution+2,0);
            count[i][j].resize(wf.wallsegments[j].length()/resolution+2,0);
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
                Eigen::Vector3d p = projectOntoFloorplan(x, y, *cam, wf);
                if (p[0] >= 0) {
                    int h = (p[2] - wf.floorplane)/resolution;
                    if (h >= hist.size()) h = hist.size()-1;
                    if (h < 0) h = 0;

                    int xx = p[1]/resolution;
                    if (xx < 0) xx = 0;
                    else if (xx >= hist[h][p[0]].size()) xx = hist[h][p[0]].size()-1;
                    count[h][p[0]][xx]++;

                    if ((!isinf(img[idx]) && img[idx] > edgethreshold) ||
                        (!isinf(img[idx+2]) && img[idx+2] > edgethreshold))
                    {
                        float v = 0;
                        if (wf.wallsegments[p[0]].direction == 0 && !isinf(img[idx]))
                            v = img[idx];
                        if (wf.wallsegments[p[0]].direction == 1 && !isinf(img[idx+2]))
                            v = img[idx+2];
                        //hist[h][p[0]][xx] = max(hist[h][p[0]][xx], log(v+1));
                        //hist[h][p[0]][xx] += log(v+1);
                        hist[h][p[0]][xx] += 1;
                        //hist[h][p[0]][xx] = max(hist[h][p[0]][xx], 1.f);
                        //cout << x << " " << y << ": " << p[2] << "(" << h << " " << (p[2] - wf.floorplane) << ") " << p[0] << " " << p[1] << "(" << xx  << ")" << endl;
                        //cout << x << "," << y << "; " << endl;
                    }
                }
            }
        }
        cout << "Done " << i << "/" << imgr->size() << endl;
    }

    /*int h = hist.size();
    for (int z = 0; z < 4; z++) {
        int w = hist[0][z].size();
        float* dbgimg = new float[w*h*3];
        float* pp = dbgimg;
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                *pp++ = hist[i][z][j];
                *pp++ = hist[i][z][j];
                *pp++ = hist[i][z][j];
            }
        }
        string filename = "dbg" + to_string(z) + ".exr";
        ImageIO::writeExrImage(filename, dbgimg, w, h);
    }*/

    double best = 0;
    vector<double> v;
    for (int i = 0; i < hist.size(); i++) {
        double weightedge = 0;
        double cellsedge = 0;
        double weightobserved = 0;
        double cellsobserved = 0;
        for (int j = 0; j < hist[i].size(); j++) {
            for (int k = 0; k < hist[i][j].size(); k++) {
                weightedge += hist[i][j][k]/count[i][j][k];
                cellsedge += hist[i][j][k]?1:0;
                weightobserved += count[i][j][k];
                cellsobserved += count[i][j][k]?1:0;
            }
        }
        if (cellsobserved == 0) {
            //printf("%d (%.4f): 0 0 0\n", i, i*resolution);
        } else {
            v.push_back(weightedge/cellsobserved);
            //printf("%d (%.4f): %.2f\n", i, i*resolution, v);
        }
    }
    for (int i = 1; i < v.size()-1; i++) {
        if (v[i] > v[i-1] && v[i] >= v[i+1]) {
            bbh = i*resolution;
            break;
        }
    }
    cout << "Best: " << best << " " << bbh << endl;
    bbd = 0;
    bbcolor = roommodel::Color(0.5,0.4,0.3);
    solved = true;
}
