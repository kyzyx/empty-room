#ifndef _HEMICUBE_RENDERER_H
#define _HEMICUBE_RENDERER_H

#include "datamanager/imagemanager.h"
#include "datamanager/material.h"
#include "rendermanager.h"
#include "lighting/light.h"

class SampleData {
    public:
        Material radiosity;
        Material netIncoming;
        float fractionUnknown;
        float fractionDirect;
        std::vector<Light*> lightamount;
        int vertexid;
};


const int LIGHT_TYPESHIFT = 5;
const int LIGHT_IDMASK = (1 << LIGHT_TYPESHIFT)-1;
inline unsigned int LIGHTID(unsigned int lightinfo) { return lightinfo & LIGHT_IDMASK; }
inline unsigned int LIGHTTYPE(unsigned int lightinfo) {
    static int COMPRESSEDLIGHTTYPES[] =
    {
        LIGHTTYPE_NULL,
        LIGHTTYPE_ENVMAP,
        LIGHTTYPE_SH,
        LIGHTTYPE_AREA,
        LIGHTTYPE_ENVMAP | LIGHTTYPE_POINT,
        LIGHTTYPE_ENVMAP | LIGHTTYPE_LINE,
        LIGHTTYPE_SH | LIGHTTYPE_POINT,
        LIGHTTYPE_SH | LIGHTTYPE_LINE,
    };
    return COMPRESSEDLIGHTTYPES[lightinfo >> LIGHT_TYPESHIFT];
}

enum {
    CELLTYPE_NONE,
    CELLTYPE_UNOBSERVED,
    CELLTYPE_LIGHT,
};

class HemicubeRenderer {
    public:
        HemicubeRenderer(RenderManager* rm, int hemicubeResolution=150);
        void computeSamples(
                std::vector<SampleData>& data,
                std::vector<int> indices,
                int numsamples,
                std::vector<Light*> lights,
                double discardthreshold=1,
                std::vector<float*>* images=NULL,
                boost::function<void(int)> cb=NULL
                );
        void renderHemicube(const R3Point& p, const R3Vector& n,
                Material& m, std::vector<Light*>& lights,
                float& fractionUnknown, float& fractionDirect,
                float* color, float* light);
        int getHemicubeResolution() const { return res; }
        SampleData computeSample(int n, std::vector<Light*> lights, float* radimage, float* lightimage);
        SampleData computeSample(const R3Point& p, const R3Vector& n, std::vector<Light*> lights, float* radimage, float* lightimage);
        void weightTopHemicube(float* img, float factor=1) const;
        void weightSideHemicube(float* img, float factor=1) const;

        void render(const CameraParams* cam, float* image, int mode=VIEW_AVERAGE);
    private:
        void renderFace(const R3Point& p,
                const R3Vector& towards, const R3Vector& up,
                float* image, int mode);
        int processHemicubeCell(
                const R3Point& p, const R3Vector& n, const R3Vector& up,
                float weight, float* image, float* light,
                Material& m, std::vector<Light*>& lights,
                int i, int j);

        void computeHemicubeFF();

        RenderManager* rendermanager;
        int res;
        float** topHemicubeFF;
        float** sideHemicubeFF;
};
#endif
