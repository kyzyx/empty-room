#ifndef _SOLVER_H
#define _SOLVER_H
#include <vector>

#include "hemicuberenderer.h"
#include "material.h"
#include "mesh.h"

class InverseRender {
    public:
        InverseRender(Mesh* m, int nlights, int hemicubeResolution=150)
            : hr(m, hemicubeResolution), mesh(m), numlights(nlights), images(NULL)
        {
            lights.resize(numlights);
        }
        void solve(std::vector<SampleData>& data);
        void computeSamples(
                std::vector<SampleData>& data,
                std::vector<int> indices,
                int numsamples,
                double discardthreshold=1,
                bool saveImages=true);

        void writeVariablesMatlab(std::vector<SampleData>& data, std::string filename);
        void writeVariablesBinary(std::vector<SampleData>& data, std::string filename);
        void loadVariablesBinary(std::vector<SampleData>& data, std::string filename);
    private:
        bool calculateWallMaterialFromUnlit(std::vector<SampleData>& data);
        bool solveLights(std::vector<SampleData>& data);
        bool solveMaterials(std::vector<SampleData>& data);

        HemicubeRenderer hr;
    public:
        float** images;
        Mesh* mesh;
        int numlights;
        std::vector<Material> lights;
        Material wallMaterial;
};

#endif
