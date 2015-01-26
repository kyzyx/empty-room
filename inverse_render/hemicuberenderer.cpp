#include <GL/glew.h>

#include "hemicuberenderer.h"
#include <iostream>
#include <stdexcept>

#define MAX_LIGHTS 127

using namespace std;

inline float labelToFloat(uint16_t l) { return l/128.; }
inline float floatToLabel(float l) { return (uint16_t) (l*128+0.5); }

HemicubeRenderer::HemicubeRenderer(const MeshManager* m, int hemicubeResolution)
    : mesh(m), res(hemicubeResolution)
{
    computeHemicubeFF();
    if (!setupRasterizer()) {
        throw runtime_error("Error initializing OpenGL textures!");
    }
    if (!setupMesh()) {
        throw runtime_error("Error initializing OpenGL mesh!");
    }
    if (!setupMeshColors()) {
        throw runtime_error("Error initializing OpenGL mesh colors!");
    }
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
            sideHemicubeFF[o+i-1][o+j-1] = d*d*z/(M_PI*denom*denom);
            sideHemicubeFF[o+i-1][o-j] = sideHemicubeFF[o+i-1][o+j-1];
            sideHemicubeFF[o-i][o-j]   = sideHemicubeFF[o+i-1][o+j-1];
            sideHemicubeFF[o-i][o+j-1] = sideHemicubeFF[o+i-1][o+j-1];
        }
    }
}
bool HemicubeRenderer::setupMesh() {
    int varraysize = 3*3*mesh->NFaces();
    float* vertices = new float[varraysize];
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    int v = 0;
    for (int i = 0; i < mesh->NFaces(); ++i) {
        R3Point p = mesh->VertexPosition(mesh->VertexOnFace(i,0));
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
        p = mesh->VertexPosition(mesh->VertexOnFace(i,1));
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
        p = mesh->VertexPosition(mesh->VertexOnFace(i,2));
        vertices[v++] = p[0];
        vertices[v++] = p[1];
        vertices[v++] = p[2];
    }
    glBufferData(GL_ARRAY_BUFFER,
            varraysize*sizeof(float),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
    return true;
}
bool HemicubeRenderer::setupMeshColors() {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    float* vertices = new float[6*3*mesh->NFaces()];
    memset(vertices, 0, sizeof(float)*6*3*mesh->NFaces());
    for (int i = 0; i < mesh->NFaces(); ++i) {
        // If any vertex has no samples, discard this face
        bool valid = true;
        // If all vertices are the same light vertices, this is a light face
        int light = 0;
        for (int j = 0; j < 3; ++j) {
            int n = mesh->VertexOnFace(i, j);
            char l = mesh->getLabel(n);
            int ind = 6*(3*i + j);
            vertices[ind+1] = labelToFloat(mesh->getLabel(n,1));
            vertices[ind+2] = 1;
            if (mesh->getVertexSampleCount(n) == 0) {
                valid = false;
            }
            if (light == 0 && l > 0) light = l;
            else if (light > 0 && l == 0) light = -1; // Half light
            else if (light > 0 && l != light) light = -2; // Differing lights, should never happen
        }
        if (!valid) continue;
        for (int j = 0; j < 3; ++j) {
            int ind = 6*(3*i + j);
            if (light > 0) {
                vertices[ind+0] = light/(float)MAX_LIGHTS;
                vertices[ind+2] = 0;
            } else if (light != -1) {
                vertices[ind+2] = 0;
                int n = mesh->VertexOnFace(i,j);
                Material m = mesh->getVertexColor(n);
                vertices[ind+3] = m.r;
                vertices[ind+4] = m.g;
                vertices[ind+5] = m.b;
            }
        }
    }
    glBufferData(GL_ARRAY_BUFFER, 6*3*mesh->NFaces()*sizeof(float),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
    return true;
}
bool HemicubeRenderer::setupRasterizer() {
    glClampColor(GL_CLAMP_READ_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_VERTEX_COLOR, GL_FALSE);
    glClampColor(GL_CLAMP_FRAGMENT_COLOR, GL_FALSE);

    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, res, res, 0, GL_RGBA, GL_FLOAT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glGenRenderbuffers(1, &fbo_z);
    glBindRenderbuffer(GL_RENDERBUFFER_EXT, fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, res, res);
    glBindRenderbuffer(GL_RENDERBUFFER_EXT, 0);

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    //glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER_EXT, fbo_rgb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, fbo_z);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
        cout << "Error creating frame buffer" << endl;
        return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
    return true;
}

void HemicubeRenderer::renderMeshOGL(bool light) const {
    glDepthMask(true);
    glClearColor(0.,0.,0.,0.);
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glColorPointer(3, GL_FLOAT, 6*sizeof(float), (void*) (light?3*sizeof(float):0));
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, 3*sizeof(float), 0);
    glDrawArrays(GL_TRIANGLES, 0, mesh->NFaces()*3);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}

float HemicubeRenderer::renderHemicube(
        const R3Point& p,
        const R3Vector& n,
        Material& m,
        vector<float>& lightareas,
        float* image, float* light)
{
    double blank = 0;
    if (image == NULL) image = new float[3*res*res];
    if (light == NULL) light = new float[3*res*res];
    R3Point pp = p + 0.0001*n;
    R3Vector x = abs(n[0])>abs(n[1])?R3yaxis_vector:R3xaxis_vector;
    x.Cross(n);
    x.Normalize();
    R3Vector y = n%x;
    R3Vector orientations[] = {x,-x,y,-y};
    for (int o = 0; o < 4; ++o) {
        renderFace(pp, orientations[o], n, image, true);
        renderFace(pp, orientations[o], n, light, false);
        for (int i = 0; i < res/2; ++i) {
            for (int j = 0; j < res; ++j) {
                if (light[3*(i*res+j)] != 0 &&
                    light[3*(i*res+j)+2] == 0)
                {
                    int lightid = light[3*(i*res+j)]*MAX_LIGHTS - 1;
                    if (lightid >= lightareas.size()) lightareas.resize(lightid+1);
                    lightareas[lightid] += sideHemicubeFF[i][j];
                }  else if (light[3*(i*res+j)] == 0 &&
                            light[3*(i*res+j)+2] != 0)
                {
                    blank += sideHemicubeFF[i][j];
                } else {
                    m.r += sideHemicubeFF[i][j]*image[3*(i*res+j)];
                    m.g += sideHemicubeFF[i][j]*image[3*(i*res+j)+1];
                    m.b += sideHemicubeFF[i][j]*image[3*(i*res+j)+2];
                }
            }
        }
    }
    renderFace(pp, n, y, image, true);
    renderFace(pp, n, y, light, false);
    for (int i = 0; i < res; ++i) {
        for (int j = 0; j < res; ++j) {
            if (light[3*(i*res+j)] != 0 &&
                light[3*(i*res+j)+2] == 0)
            {
                int lightid = light[3*(i*res+j)]*MAX_LIGHTS - 1;
                if (lightid >= lightareas.size()) lightareas.resize(lightid+1);
                lightareas[lightid] += topHemicubeFF[i][j];
            }  else if (light[3*(i*res+j)] == 0 &&
                        light[3*(i*res+j)+2] != 0)
            {
                blank += topHemicubeFF[i][j];
            } else {
                m.r += topHemicubeFF[i][j]*image[3*(i*res+j)];
                m.g += topHemicubeFF[i][j]*image[3*(i*res+j)+1];
                m.b += topHemicubeFF[i][j]*image[3*(i*res+j)+2];
            }
        }
    }
    return blank;
}
void HemicubeRenderer::renderFace(const R3Point& p,
        const R3Vector& towards, const R3Vector& up,
        float* image, bool colorimage)
{
    render(p, towards, up, 90, res, res, image, colorimage);
}
void HemicubeRenderer::render(const CameraParams* cam, float* image, bool colorimage)
{
    render(cam->pos, cam->towards, cam->up, cam->fov, cam->width, cam->height, image, colorimage);
}
void HemicubeRenderer::render(
        const R3Point& p,
        const R3Vector& towards,
        const R3Vector& up,
        const double fov,
        const int width,
        const int height,
        float* image,
        bool colorimage)
{
    glDisable(GL_CULL_FACE);
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, width/(double) height, 0.001, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    R3Point at = p + towards;
    gluLookAt(p[0], p[1], p[2], at[0], at[1], at[2], -up[0], -up[1], -up[2]); // OpenGL is lower left origin
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
    renderMeshOGL(colorimage);
    glReadPixels(0,0,width,height,GL_RGB,GL_FLOAT,(void*)image);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
}
void HemicubeRenderer::computeSamples(
        vector<SampleData>& data,
        vector<int> indices,
        int numsamples,
        double discardthreshold,
        float** images)
{
    float* lightimage;
    float* radimage;
    if (images) {
        for (int i = 0; i < numsamples; ++i) {
            images[2*i] = new float[3*res*res];
            images[2*i+1] = new float[3*res*res];
        }
    } else {
        lightimage = new float[3*res*res];
        radimage = new float[3*res*res];
    }

    default_random_engine generator;
    uniform_int_distribution<int> dist(0, indices.size());
    for (int i = 0; i < numsamples; ++i) {
        int n;
        do {
            n = dist(generator);
        } while (mesh->getLabel(indices[n]) > 0 || mesh->getVertexSampleCount(indices[n]) == 0);

        SampleData sd;
        sd.vertexid = indices[n];
        sd.radiosity = mesh->getVertexColor(indices[n]);
        if (images) {
            radimage = images[2*i];
            lightimage = images[2*i+1];
        }
        sd.fractionUnknown = renderHemicube(
                mesh->VertexPosition(indices[n]),
                mesh->VertexNormal(indices[n]),
                sd.netIncoming, sd.lightamount, radimage, lightimage
        );
        if (sd.fractionUnknown > discardthreshold) {
            --i;
            continue;
        }
        data.push_back(sd);
        if (i%10 == 9) cout << "Rendered " << i+1 << "/" << numsamples << endl;
    }
    cout << "Done sampling" << endl;
}

void HemicubeRenderer::createLabelImage(const CameraParams* cam, void* image) {
    int w = cam->width;
    int h = cam->height;
    char* ret = (char*) image;
    float* rendered = new float[w*h*3];
    render(cam, rendered, false);
    for (int i = 0; i < w*h; ++i) {
        ret[i] = floatToLabel(rendered[3*i+1]);
    }
    delete [] rendered;
}
void HemicubeRenderer::createAllLabelImages(ImageManager* imgr, boost::function<void(int)> cb) {
    for (int i = 0; i < imgr->size(); ++i) {
        if (cb) cb(100*i/imgr->size());
        createLabelImage(imgr->getCamera(i), imgr->getImageWriteable("labels", i));
    }
}
