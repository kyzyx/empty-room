#include "solver.h"
#include "wall_finder.h"
#include "rectify.h"
#include "R2ImageOld.h"
#include <deque>

using namespace std;
using namespace Eigen;

inline Vector3d gaps2eigen(const R3Vector& v) {
    return Vector3d(v[0], v[1], v[2]);
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
        const R3Plane& surface,
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
        if (cam->towards.Dot(surface.Normal()) < -cos(threshold)) {
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
    const CameraParams* cam = colorhelper->getCamera(bestimage);
    int w = cam->width;
    int h = cam->height;
    generateBinaryMask(cam, isfloor, WallFinder::LABEL_FLOOR);
    reduceToLargestCluster(isfloor, w);

    vector<Vector2d> from, to;
    // Find corners of cluster in image
    int minx = w, miny = h, maxx = 0, maxy = 0;
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            if (isfloor[i*w+j]) {
                if (i < miny) miny = i;
                if (i > maxy) maxy = i;
                if (j < minx) minx = j;
                if (j > maxx) maxx = j;
            }
        }
    }
    // Crop image to cluster
    float* uncropped = (float*) colorhelper->getImage(bestimage);
    R2Image_Old::R2Image cropped(maxx-minx+1, maxy-miny+1);
    for (int i = miny; i <= maxy; ++i) {
        for (int j = minx; j <= maxx; ++j) {
            R2Pixel pix(
                    uncropped[3*((h-i-1)*w+j)],
                    uncropped[3*((h-i-1)*w+j)+1],
                    uncropped[3*((h-i-1)*w+j)+2],
                    isfloor[i*w+j]
            );
            cropped.SetPixel(j-minx, i-miny, pix);
        }
    }

    // Trace rays to find corresponding points on plane
    vector<Vector3d> points;
    points.push_back(Vector3d(minx, miny, 0));
    points.push_back(Vector3d(minx, maxy, 0));
    points.push_back(Vector3d(maxx, maxy, 0));
    points.push_back(Vector3d(maxx, miny, 0));
    for (int i = 0; i < points.size(); ++i) {
        from.push_back(points[i].head(2) - Vector2d(minx, miny));
        R3Point p = cam->pos + cam->focal_length*cam->towards;
        p += (points[i](0) - (w-1)/2.)*cam->right;
        p -= (points[i](1) - (h-1)/2.)*cam->up;
        R3Ray ray(cam->pos, p);
        R3Point isect;
        if (!R3Intersects(ray, surface, &isect)) {
            cerr << "Unexpected error!" << endl;
            return;
        }
        for (int j = 0; j < 3; ++j) points[i](j) = isect[j];
    }

    // Determine new coordinate system on floor plane
    Vector3d v1 = points[1] - points[0];
    Vector3d v2 = -v1.cross(gaps2eigen(surface.Normal()));
    v1 *= (maxy-miny)/v1.squaredNorm();
    v2 *= v1.norm()/v2.norm();
    Vector3d origin = points[0];
    // Project points onto new coordinate system
    for (int i = 0; i < points.size(); ++i) {
        Vector3d p = points[i] - origin;
        to.push_back(Vector2d(p.dot(v2), p.dot(v1)));
    }

    // Rectify
    Matrix3d t = getRectificationMatrix(from, to);
    rectify(t, &cropped);

    // Find square region
    int best = 0;
    int bestr = -1;
    int bestc = -1;
    int* largest = new int[w*h];
    for (int i = 0; i < w; ++i) largest[i] = cropped.Pixel(i,0).Alpha() > 0;
    for (int i = 1; i < h; ++i) {
        largest[i*w] = cropped.Pixel(0,i).Alpha() > 0;
        for (int j = 1; j < w; ++j) {
            if (cropped.Pixel(j,i).Alpha() > 0) {
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
    delete [] largest;
    tex.size = best;
    tex.scale = ((points[1] - points[0]).norm()*best)/(maxy-miny);

    // Copy into image
    tex.texture = new float[3*tex.size*tex.size];
    float* next = tex.texture;
    for (int i = bestr-best+1; i <= bestr; ++i) {
        for (int j = bestc-best+1; j <= bestc; ++j) {
            for (int k = 0; k < 3; ++k) {
                *(next++) = cropped.Pixel(j,i)[k];
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
    for (int i = 0; i < tex.size*tex.size; ++i) {
        tex.texture[3*i+0] *= tot.r;
        tex.texture[3*i+1] *= tot.g;
        tex.texture[3*i+2] *= tot.b;
    }
}
