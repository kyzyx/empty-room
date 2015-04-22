#ifndef _HEMICUBE_RENDERER_H
#define _HEMICUBE_RENDERER_H

#include "camparams.h"
#include "material.h"
#include "rendermanager.h"
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
        HemicubeRenderer(RenderManager* rm, int hemicubeResolution=150);
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

        void render(const CameraParams* cam, float* image, int mode=VIEW_AVERAGE);

        void createLabelImage(const CameraParams* cam, void* image);
    private:
        void renderFace(const R3Point& p,
                const R3Vector& towards, const R3Vector& up,
                float* image, int mode);

        void computeHemicubeFF();

        RenderManager* rendermanager;
        int res;
        float** topHemicubeFF;
        float** sideHemicubeFF;
};
#endif
