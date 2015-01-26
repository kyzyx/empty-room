#ifndef _HEMICUBE_RENDERER_H
#define _HEMICUBE_RENDERER_H
#include <GL/gl.h>

#include "camparams.h"
#include "material.h"
#include "meshmanager.h"
#include "imagemanager.h"

class SampleData {
    public:
        Material radiosity;
        Material netIncoming;
        float fractionUnknown;
        std::vector<float> lightamount;
        int vertexid;
};

class HemicubeRenderer {
    public:
        HemicubeRenderer(const MeshManager* m, int hemicubeResolution=150);
        void computeSamples(
                std::vector<SampleData>& data,
                std::vector<int> indices,
                int numsamples,
                double discardthreshold=1,
                float** images=NULL);
        float renderHemicube(const R3Point& p, const R3Vector& n,
                Material& m, std::vector<float>& lightareas, float* color,
                float* light);
        int getHemicubeResolution() const { return res; }
        void renderMeshOGL(bool light) const;

        void render(const CameraParams* cam, float* image, bool colorimage);
        void render(
            const R3Point& p,
            const R3Vector& towards,
            const R3Vector& up,
            const double fov,
            const int width,
            const int height,
            float* image,
            bool colorimage);

        void createLabelImage(const CameraParams* cam, void* image);
        void createAllLabelImages(
                ImageManager* imgr,
                boost::function<void(int)> cb=NULL);
    private:
        void renderFace(const R3Point& p,
                const R3Vector& towards, const R3Vector& up,
                float* image, bool colorimage);

        const MeshManager* mesh;
        GLuint vbo, ibo, cbo;

        void computeHemicubeFF();
        bool setupRasterizer();
        bool setupMesh();
        bool setupMeshColors();

        GLuint fbo, tex, fbo_rgb, fbo_z;
        int res;
        float** topHemicubeFF;
        float** sideHemicubeFF;
};
#endif
