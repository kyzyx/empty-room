#ifndef _SOLVER_H
#define _SOLVER_H
#include <vector>

#include "R3Shapes/R3Shapes.h"
#include "datamanager/imagemanager.h"
#include "datamanager/meshmanager.h"
#include "rendering/hemicuberenderer.h"

enum {
    LOSS_L2,
    LOSS_CAUCHY,
    LOSS_HUBER,
};

class InverseRender {
    public:
        InverseRender(MeshManager* m, int hemicubeResolution=150, boost::function<void(int)> callback=NULL)
            : mesh(m),
              lossfn(LOSS_L2), scale(1),
              cb(callback)
        {
            rm = new RenderManager(m);
            rm->setupMeshColors();
            rm->precalculateAverageSamples();
            hr = new HemicubeRenderer(rm, hemicubeResolution);
            setupLightParameters(m);
        }
        ~InverseRender() {
            if (hr) delete hr;
            if (rm) delete rm;
            for (int ch = 0; ch < numchannels; ch++) {
                for (auto it : lights[ch]) delete it;
            }
        }
        void solve(std::vector<SampleData>& data, double reglambda=0);
        void solveTexture(
                std::vector<SampleData>& data,
                ImageManager* imagemanager,
                const R3Plane& surface,
                Texture& tex,
                int label);
        void computeSamples(
                std::vector<SampleData>& data,
                std::vector<int> indices,
                int numsamples,
                double discardthreshold=1,
                bool saveImages=true,
                boost::function<void(int)> callback=NULL);
        SampleData computeSample(int idx, float* color=NULL, float* aux=NULL) {
            return hr->computeSample(idx, lights[0], color, aux);
        }

        void writeVariablesMatlab(std::vector<SampleData>& data, std::string filename);
        void writeVariablesBinary(std::vector<SampleData>& data, std::string filename);
        void loadVariablesBinary(std::vector<SampleData>& data, std::string filename);

        void setLossFunction(int fn) { lossfn = fn; }
        void setLossFunctionScale(double s) { scale = s; }

        RenderManager* getRenderManager() { return rm; }
    private:
        // Texture recovery helpers
        Material computeAverageMaterial(
                std::vector<SampleData>& data,
                std::vector<std::vector<Light*> >& lightintensies);
        double generateBinaryMask(const CameraParams* cam, const char* labelimage, std::vector<bool>& mask, int label);
        void setupLightParameters(MeshManager* m);

        HemicubeRenderer* hr;
        RenderManager* rm;

        int lossfn;
        double scale;

        boost::function<void(int)> cb;
    public:
        std::vector<float*> images;
        MeshManager* mesh;
        std::vector<std::vector<Light*> > lights;
        Material wallMaterial;
        std::vector<Material> materials;
        const int numchannels = 3;
};

#endif
