#include <GL/glew.h>
#include "solver.h"
#include <random>
#define MAX_LIGHTS 10

using namespace std;

void Material::print() {
    printf("(%.3f,%.3f,%.3f)",r, g, b);
}
void InverseRender::calculate(vector<int> indices, int numsamples) {
    if (!setupRasterizer()) {
        return;
    }

    // Initial wall material guess - average all
    wallMaterial = Material(0,0,0);
    float total = 0;
    int numlights = MAX_LIGHTS;
    for (int i = 0; i < indices.size(); ++i) {
        for (int j = 0; j < mesh->samples[indices[i]].size(); ++j) {
            float s = abs(mesh->samples[indices[i]][j].dA);
            wallMaterial.r += mesh->samples[indices[i]][j].r*s;
            wallMaterial.g += mesh->samples[indices[i]][j].g*s;
            wallMaterial.b += mesh->samples[indices[i]][j].b*s;
            total += s;
        }
    }
    if (total > 0) {
        wallMaterial.r /= total*255;
        wallMaterial.g /= total*255;
        wallMaterial.b /= total*255;
    }
    cout << "Calculated average wall material: ";
    wallMaterial.print();
    cout << endl;

    images = new unsigned char*[2*numsamples];
    default_random_engine generator;
    uniform_int_distribution<int> dist(0, indices.size());
    for (int i = 0; i < numsamples; ++i) {
        int n;
        do {
            n = dist(generator);
        } while (mesh->samples[indices[n]].size() == 0);

        SampleData sd;
        sd.lightamount.resize(numlights);
        sd.vertexid = indices[n];
        sd.radiosity = Material(0,0,0);
        total = 0;
        for (int j = 0; j < mesh->samples[indices[n]].size(); ++j) {
            float s =     abs(mesh->samples[indices[n]][j].dA);
            sd.radiosity.r += mesh->samples[indices[n]][j].r*s;
            sd.radiosity.g += mesh->samples[indices[n]][j].g*s;
            sd.radiosity.b += mesh->samples[indices[n]][j].b*s;
            total += s;
        }
        sd.radiosity.r /= total*255;
        sd.radiosity.g /= total*255;
        sd.radiosity.b /= total*255;

        images[2*i] = new unsigned char[3*res*res];
        images[2*i+1] = new unsigned char[3*res*res];
        sd.fractionUnknown = renderHemicube(
                mesh->getMesh()->VertexPosition(mesh->getMesh()->Vertex(indices[n])),
                mesh->getMesh()->VertexNormal(mesh->getMesh()->Vertex(indices[n])),
                sd.netIncoming, sd.lightamount, images[2*i], images[2*i+1]
        );
        data.push_back(sd);

        cout << n << "(" << sd.fractionUnknown << "): ";
        sd.radiosity.print();
        cout << " = p*";
        sd.netIncoming.print();
        for (int k = 0; k < sd.lightamount.size(); ++k) {
            if (sd.lightamount[k] > 0) cout << " + p*L_" << k << "*" << sd.lightamount[k];
        }
        cout << endl;
    }

    bool converged = false;
    while (!converged) {
        converged = solveLights();
        converged = converged && solveMaterials();
    }
}

bool InverseRender::setupRasterizer() {
    glGenRenderbuffers(1, &fbo_rgb);
    glBindRenderbuffer(GL_RENDERBUFFER_EXT, fbo_rgb);
    glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_RGBA8, res, res);
    glGenRenderbuffers(1, &fbo_z);
    glBindRenderbuffer(GL_RENDERBUFFER_EXT, fbo_z);
    glRenderbufferStorage(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, res, res);
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER_EXT, fbo_rgb);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, fbo_z);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
        cout << "Error creating frame buffer" << endl;
        return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
    return true;
}

bool InverseRender::solveLights() {
    // FIXME
    return true;
}
bool InverseRender::solveMaterials() {
    // FIXME
    return true;
}
void InverseRender::computeHemicubeFF() {
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
float InverseRender::renderHemicube(
        const R3Point& p,
        const R3Vector& n,
        Material& m,
        vector<float>& lightareas,
        unsigned char* image, unsigned char* light)
{
    float blank = 0;
    if (image == NULL) image = new unsigned char[3*res*res];
    if (light == NULL) light = new unsigned char[3*res*res];
    R3Point pp = p + 0.0001*n;
    R3Vector x = R3yaxis_vector;
    x.Cross(n);
    R3Vector y = x%n;
    R3Vector orientations[] = {x,-x,y,-y};
    for (int o = 0; o < 4; ++o) {
        renderFace(pp, orientations[o], n, image, true);
        renderFace(pp, orientations[o], n, light, false);
        for (int i = res/2; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                if (light[3*(i*res+j)] != 0 &&
                        light[3*(i*res+j)+1] == 0 &&
                        light[3*(i*res+j)+2] == 0)
                {
                    lightareas[light[3*(i*res+j)]-1] += sideHemicubeFF[i][j];
                }  else if (light[3*(i*res+j)] == 0 &&
                        light[3*(i*res+j)+1] == 0 &&
                        light[3*(i*res+j)+2] != 0)
                {
                    blank += sideHemicubeFF[i][j];
                } else {
                    m.r += sideHemicubeFF[i][j]*image[3*(i*res+j)]/255.;
                    m.g += sideHemicubeFF[i][j]*image[3*(i*res+j)+1]/255.;
                    m.b += sideHemicubeFF[i][j]*image[3*(i*res+j)+2]/255.;
                }
            }
        }
    }
    renderFace(pp, n, y, image, true);
    renderFace(pp, n, y, light, false);
    for (int i = 0; i < res; ++i) {
        for (int j = 0; j < res; ++j) {
            if (light[3*(i*res+j)] != 0 &&
                    light[3*(i*res+j)+1] == 0 &&
                    light[3*(i*res+j)+2] == 0)
            {
                lightareas[light[3*(i*res+j)]-1] += topHemicubeFF[i][j];
            }  else if (light[3*(i*res+j)] == 0 &&
                    light[3*(i*res+j)+1] == 0 &&
                    light[3*(i*res+j)+2] != 0)
            {
                blank += topHemicubeFF[i][j];
            } else {
                m.r += topHemicubeFF[i][j]*image[3*(i*res+j)]/255.;
                m.g += topHemicubeFF[i][j]*image[3*(i*res+j)+1]/255.;
                m.b += topHemicubeFF[i][j]*image[3*(i*res+j)+2]/255.;
            }
        }
    }
    return blank;
}
void InverseRender::renderFace(const R3Point& p,
        const R3Vector& towards, const R3Vector& up,
        unsigned char* image, bool colorimage)
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
    glReadPixels(0,0,res,res,GL_RGB,GL_UNSIGNED_BYTE,(void*)image);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
}
