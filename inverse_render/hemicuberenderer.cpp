#include <GL/glew.h>

#include "hemicuberenderer.h"
#include <iostream>
#include <stdexcept>

using namespace std;

HemicubeRenderer::HemicubeRenderer(const Mesh* m, int hemicubeResolution)
    : mesh(m), res(hemicubeResolution)
{
    computeHemicubeFF();
    if (!setupRasterizer()) {
        throw runtime_error("Error initializing OpenGL textures!");
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

float HemicubeRenderer::renderHemicube(
        const R3Point& p,
        const R3Vector& n,
        Material& m,
        vector<float>& lightareas,
        float* image, float* light)
{
    float blank = 0;
    if (image == NULL) image = new float[3*res*res];
    if (light == NULL) light = new float[3*res*res];
    R3Point pp = p + 0.0001*n;
    R3Vector x = R3yaxis_vector;
    x.Cross(n);
    x.Normalize();
    R3Vector y = x%n;
    R3Vector orientations[] = {x,-x,y,-y};
    for (int o = 0; o < 4; ++o) {
        renderFace(pp, orientations[o], n, image, true);
        renderFace(pp, orientations[o], n, light, false);
        for (int i = res/2; i < res; ++i) {
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
    glDisable(GL_CULL_FACE);
    glViewport(0,0,res,res);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(90, 1., 0.001, 100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    R3Point at = p + towards;
    gluLookAt(p[0], p[1], p[2], at[0], at[1], at[2], up[0], up[1], up[2]);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
    mesh->renderOGL(colorimage);
    glReadPixels(0,0,res,res,GL_RGB,GL_FLOAT,(void*)image);
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
        } while (mesh->labels[indices[n]] > 0 || mesh->samples[indices[n]].size() == 0);

        SampleData sd;
        sd.vertexid = indices[n];
        sd.radiosity = Material(0,0,0);
        double total = 0;
        for (int j = 0; j < mesh->samples[indices[n]].size(); ++j) {
            float s = abs(mesh->samples[indices[n]][j].dA);
            sd.radiosity.r += mesh->samples[indices[n]][j].r*s;
            sd.radiosity.g += mesh->samples[indices[n]][j].g*s;
            sd.radiosity.b += mesh->samples[indices[n]][j].b*s;
            total += s;
        }
        sd.radiosity /= total;
        if (images) {
            radimage = images[2*i];
            lightimage = images[2*i+1];
        }
        sd.fractionUnknown = renderHemicube(
                mesh->getMesh()->VertexPosition(mesh->getMesh()->Vertex(indices[n])),
                mesh->getMesh()->VertexNormal(mesh->getMesh()->Vertex(indices[n])),
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
