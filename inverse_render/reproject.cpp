#include "reproject.h"

#include <iostream>
inline bool isBlack(int x, int y, int w, const char* data) {
    return data[3*(x+y*w)] == 0 && data[3*(x+y*w)+1] == 0 && data[3*(x+y*w)+2] == 0;
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
        mesh.addLabel(j, 2);
        // Check if vertex is in camera field of view
        R3Vector proj = p - cam->pos;
        double vx = proj.Dot(cam->right);
        double vy = proj.Dot(cam->up);
        if (vx > maxx || vx < -maxx || vy > maxy || vy < -maxy) continue;
        mesh.addLabel(j, 3);
        // Check if vertex intersects mesh
        R3MeshIntersection isect;
        mesh.getSearchTree()->FindIntersection(ray, isect, 0, d-0.000001);
        if (isect.t < d && isect.type != R3_MESH_NULL_TYPE) continue;
        mesh.addLabel(j, 4);

        int xx = vx*cam->width/(2*maxx) + cam->width/2 + 0.5;
        int yy = vy*cam->height/(2*maxy) + cam->height/2 + 0.5;
        // If light, label light
        if (!isBlack(xx, yy, cam->width, light)) {
            mesh.addLabel(j, 1);
        }
        // Compute direction, add sample
        Sample s;
        copyColorToSample(xx, yy, cam->width, color, s);
        R3Vector outgoing = -ray.Vector();
        copyVectorToSample(outgoing, s);
        // Using dA to dA form factor, scaled by M_PI*pixelsize
        s.dA = vhat.Dot(cam->towards)*vhat.Dot(m->VertexNormal(m->Vertex(j)))/(d*d);
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
