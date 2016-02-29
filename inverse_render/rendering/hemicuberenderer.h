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

enum LightType {
    LIGHTTYPE_AREA=0,
    LIGHTTYPE_SH=1,
    LIGHTTYPE_ENVMAP=2,
    NUMLIGHTTYPES,
};

const int LIGHT_TYPESHIFT = 6;
const int LIGHT_IDMASK = (1 << LIGHT_TYPESHIFT)-1;
inline int LIGHTID(int lightinfo) { return lightinfo & LIGHT_IDMASK; }
inline int LIGHTTYPE(int lightinfo) { return lightinfo >> LIGHT_TYPESHIFT; }
const int NUM_SH_BANDS = 5;
const int ENVMAP_RES = 4;
const int LightTypeNumCoefficients[] =
{
    1,                            // LIGHTTYPE_AREA
    NUM_SH_BANDS*NUM_SH_BANDS,    // LIGHTTYPE_SH
    6*ENVMAP_RES*ENVMAP_RES,      // LIGHTTYPE_ENVMAP
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
        void renderHemicube(const R3Point& p, const R3Vector& n,
                Material& m, std::vector<std::vector<float> >& lightareas, float& fractionUnknown,
                float* color, float* light);
        int getHemicubeResolution() const { return res; }
        SampleData computeSample(int n, float* radimage, float* lightimage);
        void weightTopHemicube(float* img, float factor=1) const;
        void weightSideHemicube(float* img, float factor=1) const;

        void render(const CameraParams* cam, float* image, int mode=VIEW_AVERAGE);
    private:
        void renderFace(const R3Point& p,
                const R3Vector& towards, const R3Vector& up,
                float* image, int mode);
        bool processHemicubeCell(
                const R3Point& p, const R3Vector& n, const R3Vector& up,
                float weight, float* image, float* light,
                Material& m, std::vector<std::vector<float> >& lightareas,
                int i, int j);

        void computeHemicubeFF();

        RenderManager* rendermanager;
        int res;
        float** topHemicubeFF;
        float** sideHemicubeFF;
};
#endif
