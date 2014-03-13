#include <GL/glew.h>
#include "solver.h"
#include <random>

using namespace std;

void InverseRender::calculate(vector<int> indices, int numsamples) {
    if (!setupRasterizer()) {
        return;
    }

    // Initial wall material guess - average all
    wallMaterial = Material(0,0,0);
    float total = 0;
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

    default_random_engine generator;
    uniform_int_distribution<int> dist(0, indices.size());
    for (int i = 0; i < numsamples; ++i) {
        int n = dist(generator);
        SampleData sd;
        sd.radiosity = Material(0,0,0);
        total = 0;
        for (int j = 0; j < mesh->samples[indices[i]].size(); ++j) {
            float s = abs(mesh->samples[indices[i]][j].dA);
            sd.radiosity.r += mesh->samples[indices[n]][j].r*s;
            sd.radiosity.g += mesh->samples[indices[n]][j].g*s;
            sd.radiosity.b += mesh->samples[indices[n]][j].b*s;
            total += s;
        }
        sd.radiosity.r /= total*255;
        sd.radiosity.g /= total*255;
        sd.radiosity.b /= total*255;

        renderHemicube(
                mesh->getMesh()->VertexPosition(mesh->getMesh()->Vertex(n)),
                mesh->getMesh()->VertexNormal(mesh->getMesh()->Vertex(n)),
                sd.netIncoming, sd.lightamount
        );
    }

    return;
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
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT) cout << "Error creating frame buffer" << endl;
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
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
    // FIXME
}
void InverseRender::renderHemicube(
        const R3Point& p,
        const R3Vector& n,
        Material& m,
        vector<float>& lightareas)
{
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
