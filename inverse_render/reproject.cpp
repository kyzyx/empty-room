#include "reproject.h"

#include <iostream>
inline bool isBlack(int x, int y, int w, const char* data) {
    return data[3*(x+y*w)] == 0 && data[3*(x+y*w)+1] == 0 && data[3*(x+y*w)+2] == 0;
}
inline bool isLight(int x, int y, int w, const float* data, double threshold) {
    return data[3*(x+y*w)] > threshold || data[3*(x+y*w)+1] > threshold || data[3*(x+y*w)+2] > threshold;
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
void reproject(const char* color, const char* light, const CameraParams* cam, Mesh& mesh) {
    R3Mesh* m = mesh.getMesh();
    double maxx = cam->width/(2*cam->focal_length);
    double maxy = cam->height/(2*cam->focal_length);
    R3Plane imageplane(cam->pos + cam->towards, cam->towards);
    for (int j = 0; j < m->NVertices(); ++j) {
        R3Vector v = m->VertexPosition(m->Vertex(j)) - cam->pos;
        double d = v.Length();
        R3Vector vhat = v/d;
        R3Ray ray(cam->pos, vhat, true);
        // Check if vertex on the right side of the camera
        R3Point p;
        if (!R3Intersects(ray, imageplane, &p)) continue;
        mesh.labels[j] = 2;
        // Check if vertex is in camera field of view
        R3Vector proj = p - cam->pos;
        double vx = proj.Dot(cam->right);
        double vy = proj.Dot(cam->up);
        if (vx > maxx || vx < -maxx || vy > maxy || vy < -maxy) continue;
        mesh.labels[j] = 3;
        // Check if vertex intersects mesh
        R3MeshIntersection isect;
        mesh.getSearchTree()->FindIntersection(ray, isect, 0, d-0.000001);
        if (isect.t < d && isect.type != R3_MESH_NULL_TYPE) continue;
        mesh.labels[j] = 4;

        int xx = vx*cam->width/(2*maxx) + cam->width/2 + 0.5;
        int yy = vy*cam->height/(2*maxy) + cam->height/2 + 0.5;
        // If light, label light
        // Compute direction, add sample
        Sample s;
        s.label = 0;
        if (!isBlack(xx, yy, cam->width, light)) {
            s.label = 1;
        }
        copyColorToSample(xx, yy, cam->width, color, s);
        R3Vector outgoing = -ray.Vector();
        copyVectorToSample(outgoing, s);
        // Using dA to dA form factor, scaled by M_PI*pixelsize
        s.dA = -vhat.Dot(cam->towards)*vhat.Dot(m->VertexNormal(m->Vertex(j)))/(d*d);
        // Use approximation that distance from point to pixel is much greater than
        // pixel size (e.g. for 54 degree fov at 640x480, each pixel is 1.7mm
        //s.dA = vhat.dot(cam->towards)/(d*d); // Solid Angle
        mesh.addSample(j, s);
    }
}
void reproject(
        const float* hdrimage,
        const CameraParams* cam,
        Mesh& mesh,
        double threshold,
        bool flip_x,
        bool flip_y)
{
    R3Mesh* m = mesh.getMesh();
    double maxx = cam->width/(2*cam->focal_length);
    double maxy = cam->height/(2*cam->focal_length);
    R3Plane imageplane(cam->pos + cam->towards, cam->towards);
    for (int j = 0; j < m->NVertices(); ++j) {
        R3Vector v = m->VertexPosition(m->Vertex(j)) - cam->pos;
        double d = v.Length();
        R3Vector vhat = v/d;
        R3Ray ray(cam->pos, vhat, true);
        // Check if vertex on the right side of the camera
        R3Point p;
        if (!R3Intersects(ray, imageplane, &p)) continue;
        mesh.labels[j] = 2;
        // Check if vertex is in camera field of view
        R3Vector proj = p - cam->pos;
        double vx = proj.Dot(cam->right);
        double vy = proj.Dot(cam->up);
        if (vx > maxx || vx < -maxx || vy > maxy || vy < -maxy) continue;
        mesh.labels[j] = 3;
        // Check if vertex intersects mesh
        R3MeshIntersection isect;
        mesh.getSearchTree()->FindIntersection(ray, isect, 0, d-0.000001);
        if (isect.t < d && isect.type != R3_MESH_NULL_TYPE) continue;
        mesh.labels[j] = 4;

        int xx = vx*cam->width/(2*maxx) + cam->width/2;
        if (flip_x) xx = cam->width - xx - 1;
        int yy = vy*cam->height/(2*maxy) + cam->height/2;
        if (flip_y) yy = cam->height - yy - 1;
        // If light, label light
        // Compute direction, add sample
        Sample s;
        s.label = 0;
        if (isLight(xx, yy, cam->width, hdrimage, threshold)) {
            s.label = 1;
        }
        copyColorToSample(xx, yy, cam->width, hdrimage, s);
        R3Vector outgoing = -ray.Vector();
        copyVectorToSample(outgoing, s);
        // Using dA to dA form factor, scaled by M_PI*pixelsize
        s.dA = -vhat.Dot(cam->towards)*vhat.Dot(m->VertexNormal(m->Vertex(j)))/(d*d);
        // Use approximation that distance from point to pixel is much greater than
        // pixel size (e.g. for 54 degree fov at 640x480, each pixel is 1.7mm
        //s.dA = vhat.dot(cam->towards)/(d*d); // Solid Angle
        mesh.addSample(j, s);
    }
}
void reproject(ColorHelper& ch, ColorHelper& lights, Mesh& mesh) {
    for (int i = 0; i < ch.size(); ++i) {
        reproject(ch.getImage(i), lights.getImage(i), ch.getCamera(i), mesh);
        std::cout << "Finished projecting image " << i << std::endl;
    }
}
void reproject(ColorHelper& hdr, Mesh& mesh, double threshold, bool flip_x, bool flip_y) {
    for (int i = 0; i < hdr.size(); ++i) {
        reproject((float*)hdr.getImage(i), hdr.getCamera(i), mesh, threshold, flip_x, flip_y);
        std::cout << "Finished projecting image " << i << std::endl;
    }
}
