#include "solver.h"
#include "wall_finder.h"
#include <deque>

using namespace std;
using namespace Eigen;

inline R3Vector eigen2gaps(Vector3f v) {
    return R3Vector(v(0), v(1), v(2));
}

int bfsLabel(vector<int>& labels, const vector<bool>& open, int w) {
    vector<bool> visited(open.size(), false);
    deque<int> q;
    int currlabel = 0;
    for (int i = 0; i < open.size(); ++i) {
        if (visited[i] || !open[i]) continue;
        q.push_back(i);
        visited[i] = true;
        labels[i] = currlabel++;
        while (!q.empty()) {
            int n = q.front(); q.pop_front();
            int x = n%w;
            int y = n/w;
            int xx[4] = {x, x, x+1, x-1};
            int yy[4] = {y+1, y-1, y, y};
            for (int n = 0; n < 4; ++n) {
                if (yy[n] < 0 || yy[n]*w >= open.size() || xx[n] < 0 || xx[n] >= w) continue;
                int idx = xx[n] + yy[n]*w;
                if (open[idx] && !visited[idx]) {
                    q.push_back(idx);
                    labels[idx] = currlabel;
                    visited[idx] = true;
                }
            }
        }
    }
    return currlabel;
}

int reduceToLargestCluster(vector<bool>& v, int w) {
    vector<int> labels(v.size());
    int numlabels = bfsLabel(labels, v, w);
    if (numlabels == 0) return 0;
    // Count cluster sizes
    vector<int> szs(numlabels, 0);
    for (int i = 0; i < v.size(); ++i) {
        if (v[i]) szs[labels[i]]++;
    }
    int largestcluster = distance(szs.begin(), max_element(szs.begin(), szs.end()));
    // Clear non-max clusters
    for (int i = 0; i < v.size(); ++i) {
        if (v[i] && labels[i] != largestcluster) v[i] = false;
    }
    return szs[largestcluster];
}

Material InverseRender::computeAverageMaterial(vector<SampleData>& data, vector<Material>& lightintensities) {
    Material avg;
    for (int i = 0; i < data.size(); ++i) {
        // Compute incident direct lighting
        Material directlighting;   // Total incoming radiance from direct light
        Material indirectlighting; // Total incoming indirect radiance
        double totallight = 0;     // Proportion of hemisphere containing direct light
        for (int j = 0; j < data[i].lightamount.size(); ++j) {
            directlighting += lightintensities[j]*data[i].lightamount[j];
            totallight += data[i].lightamount[j];
        }
        indirectlighting = data[i].netIncoming;
        // Assume unsampled regions of hemicube are equal to average
        // indirect illumination
        indirectlighting *= (1-totallight)/(1-totallight-data[i].fractionUnknown);
        avg += data[i].radiosity/(directlighting + indirectlighting);
    }
    return avg/data.size();
}

double InverseRender::generateBinaryMask(const CameraParams* cam, vector<bool>& mask, int label) {
    int w = cam->width;
    int h = cam->height;
    float labelimage[w*h*3];
    mask.resize(w*h, false);
    hr.render(cam->pos, cam->towards, cam->up, cam->fov, w, h, labelimage, false);
    int numlabelled = 0;
    for (int i = 0; i < w*h; ++i) {
        mask[i] = (labelimage[3*i+1] == label/128.);
        if (mask[i]) numlabelled++;
    }
    return numlabelled / (double) (w*h);
}

void InverseRender::solveTexture(
        vector<SampleData>& data,
        ColorHelper* colorhelper,
        Vector3f surfacenormal,
        Texture& tex)
{
    tex.size = 0;
    // Compute average reflectance
    vector<Material> truth;
    if (lights.size() > 1) truth.push_back(Material(75, 73.88, 48.78));
    truth.push_back(Material(100,100,100));
    Material avg = computeAverageMaterial(data, truth);

    // Find best image containing enough textured pixels and from a reasonably
    // direct camera pose
    int bestimage = -1;
    double bestproportion = 0;
    const double threshold = 5*M_PI/12;
    vector<bool> isfloor;
    for (int i = 0; i < colorhelper->size(); ++i) {
        const CameraParams* cam = colorhelper->getCamera(i);
        if (cam->towards.Dot(eigen2gaps(surfacenormal)) < -cos(threshold)) {
            double p;
            p = generateBinaryMask(cam, isfloor, WallFinder::LABEL_FLOOR);
            if (p == 0) continue;
            int n = reduceToLargestCluster(isfloor, cam->width);
            p = n/(double)(cam->width*cam->height);
            if (p > bestproportion) {
                bestproportion = p;
                bestimage = i;
            }
        }
    }
    if (bestimage == -1) {
        cerr << "Warning: No image with clear image of texture found" << endl;
        return;
    } else {
        cout << "Using image index " << bestimage << " to obtain texture" << endl;
    }
    generateBinaryMask(colorhelper->getCamera(bestimage), isfloor, WallFinder::LABEL_FLOOR);
    reduceToLargestCluster(isfloor, colorhelper->getCamera(bestimage)->width);
    int w = colorhelper->getCamera(bestimage)->width;
    int h = colorhelper->getCamera(bestimage)->height;

    // Find square region (and rectify?)
    int best = 0;
    int bestr = -1;
    int bestc = -1;
    int* largest = new int[w*h];
    for (int i = 0; i < w; ++i) largest[i] = isfloor[i];
    for (int i = 1; i < h; ++i) {
        largest[i*w] = isfloor[i*w];
        for (int j = 1; j < w; ++j) {
            if (isfloor[i*w+j]) {
                largest[i*w+j] = min(largest[i*w+j-1], min(largest[(i-1)*w+j], largest[(i-1)*w+j-1])) + 1;
                if (largest[i*w+j] > best) {
                    best = largest[i*w+j];
                    bestr = i;
                    bestc = j;
                }
            } else {
                largest[i*w+j] = 0;
            }
        }
    }
    tex.size = best;

    // Copy into image
    tex.texture = new float[3*tex.size*tex.size];
    float* next = tex.texture;
    for (int i = bestr-best+1; i <= bestr; ++i) {
        for (int j = bestc-best+1; j <= bestc; ++j) {
            for (int k = 0; k < 3; ++k) {
                int idx = (h-i-1)*w+j;
                *(next++) = ((float*) colorhelper->getImage(bestimage))[3*idx+k];
            }
        }
    }

    // Scale image to same average reflectance
    Material tot;
    for (int i = 0; i < tex.size*tex.size; ++i) {
        tot.r += tex.texture[3*i+0];
        tot.g += tex.texture[3*i+1];
        tot.b += tex.texture[3*i+2];
    }
    tot /= tex.size*tex.size;
    tot.r = avg.r/tot.r;
    tot.g = avg.g/tot.g;
    tot.b = avg.b/tot.b;
    cout << "Done rescaling" << endl;
    for (int i = 0; i < tex.size*tex.size; ++i) {
        tex.texture[3*i+0] *= tot.r;
        tex.texture[3*i+1] *= tot.g;
        tex.texture[3*i+2] *= tot.b;
    }
}
