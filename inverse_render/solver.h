#ifndef _SOLVER_H
#define _SOLVER_H
#include <vector>

#include "R3Shapes/R3Shapes.h"
#include "colorhelper.h"
#include "hemicuberenderer.h"
#include "material.h"
#include "mesh.h"

class InverseRender {
    public:
        InverseRender(HemicubeRenderer& renderer, int nlights)
            : hr(renderer), mesh(renderer.getMesh()), numlights(nlights), images(NULL),
              maxPercentErr(0.1), numRansacIters(1000)
        {
            lights.resize(numlights);
        }
        void solve(std::vector<SampleData>& data);
        void solveTexture(
                std::vector<SampleData>& data,
                ColorHelper* colorhelper,
                const R3Plane& surface,
                Texture& tex);
        void computeSamples(
                std::vector<SampleData>& data,
                std::vector<int> indices,
                int numsamples,
                double discardthreshold=1,
                bool saveImages=true);

        void writeVariablesMatlab(std::vector<SampleData>& data, std::string filename);
        void writeVariablesBinary(std::vector<SampleData>& data, std::string filename);
        void loadVariablesBinary(std::vector<SampleData>& data, std::string filename);

        void setNumRansacIters(int n) { numRansacIters = n; }
        void setMaxPercentErr(double d) { maxPercentErr = d; }
    private:
        // Inverse Rendering helpers
        bool calculateWallMaterialFromUnlit(std::vector<SampleData>& data);
        bool solveAll(std::vector<SampleData>& data);

        // Texture recovery helpers
        Material computeAverageMaterial(
                std::vector<SampleData>& data,
                std::vector<Material>& lightintensies);
        double generateBinaryMask(const CameraParams* cam, std::vector<bool>& mask, int label);

        HemicubeRenderer& hr;

        int numRansacIters;
        double maxPercentErr;
    public:
        float** images;
        const Mesh* mesh;
        int numlights;
        std::vector<Material> lights;
        Material wallMaterial;
};

#endif
