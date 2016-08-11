#include "hemicuberenderer.h"
#include <iostream>

#define MAX_LIGHTS 127

using namespace std;

HemicubeRenderer::HemicubeRenderer(RenderManager* rm, int hemicubeResolution)
    : rendermanager(rm), res(hemicubeResolution)
{
    computeHemicubeFF();
}
void HemicubeRenderer::computeHemicubeFF() {
    // Some redundancy, but storage is cheap
    topHemicubeFF = new float*[res];
    sideHemicubeFF = new float*[res];
    for (int i = 0; i < res; ++i) {
        topHemicubeFF[i] = new float[res];
        sideHemicubeFF[i] = new float[res];
    }
    float x = 1./res;
    float d = 2./res;
    int o = res/2;
    float tot = 0;
    for (int i = 1; i <= res/2; ++i, x += d) {
        float y = 1./res;
        for (int j = 1; j <= i; ++j, y += d) {
            float denom = x*x + y*y + 1;
            topHemicubeFF[o+i-1][o+j-1] = d*d/(M_PI*denom*denom);
            topHemicubeFF[o+i-1][o-j]   = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-i][o-j]     = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-i][o+j-1]   = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o+j-1][o+i-1] = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o+j-1][o-i]   = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-j][o-i]     = topHemicubeFF[o+i-1][o+j-1];
            topHemicubeFF[o-j][o+i-1]   = topHemicubeFF[o+i-1][o+j-1];
        }
        float z = 1./res;
        for (int j = 1; j <= res/2; ++j, z += d) {
            float denom = z*z + x*x + 1;
            sideHemicubeFF[o+i-1][o+j-1] = d*d*x/(M_PI*denom*denom);
            sideHemicubeFF[o+i-1][o-j]   = sideHemicubeFF[o+i-1][o+j-1];
            sideHemicubeFF[o-i][o+j-1]   = sideHemicubeFF[o+i-1][o+j-1];
            sideHemicubeFF[o-i][o-j]     = sideHemicubeFF[o+i-1][o+j-1];
        }
    }
}

void HemicubeRenderer::weightTopHemicube(float* img, float factor) const {
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res; j++) {
            for (int k = 0; k < 3; k++) {
                *img++ *= topHemicubeFF[i][j]*factor;
            }
        }
    }
}
void HemicubeRenderer::weightSideHemicube(float* img, float factor) const {
    for (int i = 0; i < res; i++) {
        for (int j = 0; j < res; j++) {
            for (int k = 0; k < 3; k++) {
                *img++ *= sideHemicubeFF[i][j]*factor;
            }
        }
    }
}

void HemicubeRenderer::renderHemicube(
        const R3Point& p,
        const R3Vector& n,
        Material& m,
        vector<Light*>& lights,
        float& fractionUnknown, float& fractionDirect,
        float* image, float* light)
{
    fractionUnknown = 0;
    if (image == NULL) image = new float[3*res*res];
    if (light == NULL) light = new float[3*res*res];
    R3Point pp = p + 0.0001*n;
    R3Vector x = abs(n[0])>abs(n[2])?R3zaxis_vector:R3xaxis_vector;
    x.Cross(n);
    x.Normalize();
    R3Vector y = n%x;
    R3Vector orientations[] = {x,-x,y,-y};
    for (int o = 0; o < 4; ++o) {
        renderFace(pp, orientations[o], n, image, VIEW_AVERAGE);
        renderFace(pp, orientations[o], n, light, VIEW_LABELS);
        for (int i = res/2; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                int occupied = processHemicubeCell(
                        p, orientations[o], n,
                        sideHemicubeFF[i][j], image + 3*(i*res+j), light + 3*(i*res+j),
                        m, lights, i, j);
                if (occupied == CELLTYPE_UNOBSERVED)
                    fractionUnknown += sideHemicubeFF[i][j];
                else if (occupied == CELLTYPE_LIGHT)
                    fractionDirect += sideHemicubeFF[i][j];
            }
        }
    }
    renderFace(pp, n, y, image, VIEW_AVERAGE);
    renderFace(pp, n, y, light, VIEW_LABELS);
    for (int i = 0; i < res; ++i) {
        for (int j = 0; j < res; ++j) {
                int occupied = processHemicubeCell(
                        p, n, y,
                        topHemicubeFF[i][j], image + 3*(i*res+j), light + 3*(i*res+j),
                        m, lights, i, j);
                if (occupied == CELLTYPE_UNOBSERVED)
                    fractionUnknown += topHemicubeFF[i][j];
                else if (occupied == CELLTYPE_LIGHT)
                    fractionDirect += topHemicubeFF[i][j];
        }
    }
}
int HemicubeRenderer::processHemicubeCell(
        const R3Point& p, const R3Vector& towards, const R3Vector& up,
        float weight, float* image, float* light,
        Material& m, std::vector<Light*>& lights,
        int i, int j)
{
    int visibility = ftoi(light[0]);
    int lightinfo = ftoi(light[1]);
    int lightid = LIGHTID(lightinfo);
    int lighttype = LIGHTTYPE(lightinfo);
    if (lightid > 0 && lights.size() >= lightid) {
        R3Vector x = towards%up;
        R3Vector y = up;
        double cellsize = 2./res;
        R3Vector v = (i-res/2 + 0.5)*y*cellsize + (j-res/2 + 0.5)*x*cellsize + towards;
        v.Normalize();
        v = -v;

        bool isarea = lights[lightid-1]->addLightFF(p[0], p[1], p[2], v[0], v[1], v[2], weight);
        if (isarea) return CELLTYPE_LIGHT;
        else return CELLTYPE_UNOBSERVED;
    } else if (visibility <= 0) {
        return CELLTYPE_UNOBSERVED;
    } else {
        m.r += weight*image[0];
        m.g += weight*image[1];
        m.b += weight*image[2];
        return CELLTYPE_NONE;
    }
}

void HemicubeRenderer::renderFace(const R3Point& p,
        const R3Vector& towards, const R3Vector& up,
        float* image, int mode)
{
    CameraParams cam;
    cam.pos = p;
    cam.towards = towards;
    cam.up = up;
    cam.fov = 90;
    cam.width = res;
    cam.height = res;
    render(&cam, image, mode);
}

void HemicubeRenderer::render(const CameraParams* cam, float* image, int mode)
{
    rendermanager->readFromRender(cam, image, mode, true);
}

int occluded(R3Point a, R3Point b, int lid, MeshManager* mmgr) {
    R3MeshSearchTree* st = mmgr->getSearchTree();
    int n = 1;
    do {
        const double EPSILON = 1e-6;
        R3Vector v = b - a;
        double d = v.Length();
        R3Vector vhat = v/d;
        R3Ray ray(a+vhat*EPSILON, vhat, true);
        R3MeshIntersection isect;
        st->FindIntersection(ray, isect, 0, d-2*EPSILON);
        if (isect.t < d && isect.type != R3_MESH_NULL_TYPE) {
            R3MeshVertex* v;
            if (isect.type == R3_MESH_FACE_TYPE) {
                v = st->mesh->VertexOnFace(isect.face);
            } else if (isect.type == R3_MESH_VERTEX_TYPE) {
                v = isect.vertex;
            } else if (isect.type == R3_MESH_EDGE_TYPE) {
                v = st->mesh->VertexOnEdge(isect.edge);
            }
            int vid = st->mesh->VertexID(v);
            int vlid = LIGHTID((unsigned char) mmgr->getLabel(vid, MeshManager::LABEL_CHANNEL));
            if (vlid != lid) {
                return n;
            }
            a = isect.point;
            n++;
        } else {
            return -n;
        }
    } while(true);
}

void HemicubeRenderer::computeSamples(
        vector<SampleData>& data,
        vector<int> indices,
        int numsamples,
        vector<Light*> lights,
        double discardthreshold,
        vector<float*>* images,
        boost::function<void(int)> cb)
{
    float* lightimage;
    float* radimage;
    if (!images) {
        lightimage = new float[3*res*res];
        radimage = new float[3*res*res];
    }

    for (int i = 0; i < indices.size(); i++) {
        if (cb) cb(100*i/indices.size());
        if (images) {
            radimage = new float[3*res*res];
            lightimage = new float[3*res*res];
            images->push_back(radimage);
            images->push_back(lightimage);
        }
        SampleData sd = computeSample(indices[i], lights, radimage, lightimage);
        R3Point p = rendermanager->getMeshManager()->VertexPosition(indices[i]);
        R3Vector n = rendermanager->getMeshManager()->VertexNormal(indices[i]);
        for (int j = 0; j < lights.size(); j++) {
            if (lights[j]->typeId() & LIGHTTYPE_POINT) {
                SymmetricPointLight* pointlight = (SymmetricPointLight*) sd.lightamount[j];
                R3Point lp(pointlight->getPosition(0),
                           pointlight->getPosition(1),
                           pointlight->getPosition(2));
                R3Vector v = lp - p;
                R3Vector vn = v;
                vn.Normalize();
                if (n.Dot(vn) > 0 && occluded(lp, p, j+1, rendermanager->getMeshManager()) < 0) {
                    double weight = n.Dot(vn)/v.Dot(v);
                    pointlight->addIncident(p[0], p[1], p[2], 0, 0, 0, weight/M_PI);
                }
            } else if (lights[j]->typeId() & LIGHTTYPE_LINE) {
                LineLight* linelight = (LineLight*) sd.lightamount[j];
                double weight = 0;
                double dx = linelight->getLength()/linelight->getNumSubdivs();
                R3Vector dv(linelight->getVector(0),
                            linelight->getVector(1),
                            linelight->getVector(2));
                for (int k = 0; k < linelight->getNumSubdivs(); k++) {
                    Eigen::Vector3d subpoint = linelight->getSubpoint(k);
                    R3Point lp(subpoint[0], subpoint[1], subpoint[2]);
                    R3Vector v = lp - p;
                    R3Vector vn = v;
                    vn.Normalize();
                    if (n.Dot(vn) > 0 && occluded(lp, p, j+1, rendermanager->getMeshManager()) < 0) {
                        /*
                        // Equivalently
                        R3Vector pv = v - dv*v.Dot(dv);
                        pv.Normalize();
                        weight += dx*vn.Dot(n)*(pv.dot(vn))/v.Dot(v);*/
                        double r2 = v.Dot(v);
                        weight += dx*v.Dot(n)*(dv%v).Length()/(r2*r2);
                    }
                }
                linelight->addIncident(p[0], p[1], p[2], 0, 0, 0, weight/M_PI);
            }
        }
        data.push_back(sd);
    }
}

SampleData HemicubeRenderer::computeSample(int n, vector<Light*> lights, float* radimage, float* lightimage) {
    R3Point p = rendermanager->getMeshManager()->VertexPosition(n);
    R3Vector normal = rendermanager->getMeshManager()->VertexNormal(n);
    SampleData sd = computeSample(p, normal, lights, radimage, lightimage);
    sd.radiosity = rendermanager->getMeshManager()->getMedianVertexColor(n);
    sd.vertexid = n;
    return sd;
}
SampleData HemicubeRenderer::computeSample(const R3Point& p, const R3Vector& n, std::vector<Light*> lights, float* radimage, float* lightimage) {
    SampleData sd;
    sd.netIncoming = Material();
    sd.fractionUnknown = 0;
    sd.fractionDirect = 0;
    for (int i = 0; i < lights.size(); i++) {
        Light* l = NewLightFromLightType(lights[i]->typeId());
        if (l->typeId() & LIGHTTYPE_POINT) {
            SymmetricPointLight* pl = (SymmetricPointLight*) l;
            SymmetricPointLight* ol = (SymmetricPointLight*) lights[i];
            pl->setPosition(ol->getPosition(0), ol->getPosition(1), ol->getPosition(2));
        } else if (l->typeId() & LIGHTTYPE_LINE) {
            LineLight* ll = (LineLight*) l;
            LineLight* ol = (LineLight*) lights[i];
            ll->setPosition(0, ol->getPosition(0));
            ll->setPosition(1, ol->getPosition(1));
            ll->setSymmetric(ol->isSymmetric());
        }
        sd.lightamount.push_back(l);
    }
    renderHemicube(
            p, n, sd.netIncoming, sd.lightamount, sd.fractionUnknown, sd.fractionDirect,
            radimage, lightimage
            );
    return sd;
}
