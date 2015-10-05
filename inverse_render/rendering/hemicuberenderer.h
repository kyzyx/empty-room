#ifndef _HEMICUBE_RENDERER_H
#define _HEMICUBE_RENDERER_H

#include "datamanager/imagemanager.h"
#include "datamanager/material.h"
#include "rendermanager.h"

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
                std::vector<float*>* images=NULL,
                boost::function<void(int)> cb=NULL
                );
        float renderHemicube(const R3Point& p, const R3Vector& n,
                Material& m, std::vector<float>& lightareas, float* color,
                float* light);
        int getHemicubeResolution() const { return res; }

        void render(const CameraParams* cam, float* image, int mode=VIEW_AVERAGE);
    private:
        SampleData computeSample(int n, float* radimage, float* lightimage);
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
