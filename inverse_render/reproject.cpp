#include "reproject.h"

#include <limits>
#include <iostream>
using namespace std;

const float DEPTHTOLERANCE = 0.6;

inline bool shouldDiscard(float confidence) {
    return confidence <= 0;
}
inline bool isLight(int idx, const float* data, const float* confidencemap, double threshold) {
    return (confidencemap && confidencemap[idx] < 0) || data[3*idx] > threshold || data[3*idx+1] > threshold || data[3*idx+2] > threshold;
}
inline void copyColorToSample(int x, int y, int w, const float* data, Sample& s) {
    s.r = M_PI*data[3*(x+y*w)];
    s.g = M_PI*data[3*(x+y*w)+1];
    s.b = M_PI*data[3*(x+y*w)+2];
}
inline void copyColorToSample(int x, int y, int w, const char* data, Sample& s) {
    s.r = data[3*(x+y*w)];
    s.g = data[3*(x+y*w)+1];
    s.b = data[3*(x+y*w)+2];
}
inline void copyVectorToSample(R3Vector& v, Sample& s) {
    s.x = v[0];
    s.y = v[1];
    s.z = v[2];
}

void reproject(
        const float* hdrimage,
        const float* confidencemap,
        const float* depthmap,
        const CameraParams* cam,
        MeshManager& mesh,
        double threshold,
        vector<int>& vids,
        vector<Sample>& samples,
        int16_t id,
        R3MeshSearchTree* searchtree)
{
    if (!searchtree) {
        R3MeshSearchTree st(mesh.getMesh());
        searchtree = &st;
    }
    double maxx = cam->width/(2*cam->focal_length);
    double maxy = cam->height/(2*cam->focal_length);
    R3Plane imageplane(cam->pos + cam->towards, cam->towards);
    for (int j = 0; j < mesh.NVertices(); ++j) {
        R3Vector v = mesh.VertexPosition(j) - cam->pos;
        double d = v.Length();
        R3Vector vhat = v/d;
        R3Ray ray(cam->pos, vhat, true);
        // Check if vertex on the right side of the camera
        R3Point p;
        if (!R3Intersects(ray, imageplane, &p)) continue;
        // Check if vertex is in camera field of view
        R3Vector proj = p - cam->pos;
        double vx = proj.Dot(cam->right);
        double vy = proj.Dot(cam->up);
        if (vx > maxx || vx < -maxx || vy > maxy || vy < -maxy) continue;
        // Check if vertex intersects mesh
        R3MeshIntersection isect;
        searchtree->FindIntersection(ray, isect, 0, d-0.000001);
        if (isect.t < d && isect.type != R3_MESH_NULL_TYPE) continue;

        int xx = vx*cam->focal_length + cam->width/2;
        int yy = vy*cam->focal_length + cam->height/2;
        int idx = xx + yy*cam->width;
        // Check if depth map is consistent with projection
        if (depthmap) {
            float f = depthmap[idx];
            if (!isnan(f) && f != 0 && f != numeric_limits<float>::infinity()) {
                if (abs(f-d) > DEPTHTOLERANCE)
                    continue;
            }
        }
        // If light, label light
        // Compute direction, add sample
        Sample s;
        s.label = 0;
        s.id = id;
        if (isLight(idx, hdrimage, confidencemap, threshold)) {
            s.label = 1;
        }
        if (!confidencemap || !shouldDiscard(confidencemap[idx]) || s.label == 1) {
            copyColorToSample(xx, yy, cam->width, hdrimage, s);
            if (!confidencemap) s.confidence = 1;
            else s.confidence = confidencemap[idx];
            R3Vector outgoing = -ray.Vector();
            copyVectorToSample(outgoing, s);
            // Using dA to dA form factor, scaled by M_PI*pixelsize
            s.dA = -vhat.Dot(cam->towards)*vhat.Dot(mesh.VertexNormal(j))/(d*d);
            // Use approximation that distance from point to pixel is much greater than
            // pixel size (e.g. for 54 degree fov at 640x480, each pixel is 1.7mm)
            //s.dA = vhat.dot(cam->towards)/(d*d); // Solid Angle
            samples.push_back(s);
            vids.push_back(j);
        }
    }
}

void reproject(
        const float* hdrimage,
        const float* confidencemap,
        const float* depthmap,
        const CameraParams* cam,
        MeshManager& mesh,
        double threshold,
        int16_t id,
        R3MeshSearchTree* searchtree)
{
    vector<int> vids;
    vector<Sample> samples;
    reproject(hdrimage, confidencemap, depthmap, cam, mesh, threshold, vids, samples, id, searchtree);
    for (int i = 0; i < vids.size(); ++i) {
        mesh.addSample(vids[i],samples[i]);
    }
}

void reproject(ImageManager& hdr, MeshManager& mesh, double threshold, boost::function<void(int)> cb) {
    R3MeshSearchTree searchtree(mesh.getMesh());
    for (int i = 0; i < hdr.size(); ++i) {
        reproject(
                (const float*) hdr.getImage(i),
                (const float*) hdr.getImage("confidence", i),
                (const float*) hdr.getImage("depth", i),
                hdr.getCamera(i), mesh, threshold, i, &searchtree);
        cout << "Done reprojecting " << (i+1) << "/" << hdr.size() << endl;
        if (cb) cb(((i+1)*100)/hdr.size());
    }
}
