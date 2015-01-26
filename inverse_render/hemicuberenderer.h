#ifndef _HEMICUBE_RENDERER_H
#define _HEMICUBE_RENDERER_H
#include <GL/gl.h>

#include "material.h"
#include "mesh.h"

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
        HemicubeRenderer(const Mesh* m, int hemicubeResolution=150);
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
        void render(
            const R3Point& p,
            const R3Vector& towards,
            const R3Vector& up,
            const double fov,
            const int width,
            const int height,
            float* image,
            bool colorimage);

        const Mesh* getMesh() const { return mesh; }
    private:
        void renderFace(const R3Point& p,
                const R3Vector& towards, const R3Vector& up,
                float* image, bool colorimage);

        const Mesh* mesh;
        void computeHemicubeFF();
        bool setupRasterizer();
        GLuint fbo, tex, fbo_rgb, fbo_z;
        int res;
        float** topHemicubeFF;
        float** sideHemicubeFF;
};
#endif
