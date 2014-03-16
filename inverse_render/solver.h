#ifndef _SOLVER_H
#define _SOLVER_H
#include <GL/gl.h>
#include "mesh.h"
#include <vector>

class Material {
    public:
        Material(float red, float green, float blue)
            : r(red), g(green), b(blue), texture(NULL) {;}
        Material()
            : r(0), g(0), b(0), texture(NULL) {;}
        float& operator()(int n) {
            if (n == 0) return r;
            else if (n == 1) return g;
            else if (n == 2) return b;
        }
        float r;
        float g;
        float b;
        char* texture;

        void print();
};

class InverseRender {
    public:
        InverseRender(Mesh* m, int hemicubeResolution=100)
            : mesh(m), res(hemicubeResolution), images(NULL) {
            computeHemicubeFF();
        }

        void calculate(std::vector<int> indices, int numsamples, double discardthreshold, int numlights);
        void solve();

        void renderFace(const R3Point& p,
                const R3Vector& towards, const R3Vector& up,
                unsigned char* image, bool colorimage);
        bool setupRasterizer();
        void writeVariablesMatlab(std::string filename);
        void writeVariablesBinary(std::string filename);
        void loadVariablesBinary(std::string filename);
    private:
        bool calculateWallMaterialFromUnlit();
        float renderHemicube(const R3Point& p, const R3Vector& n,
                Material& m, std::vector<float>& lightareas, unsigned char* color,
                unsigned char* light);
        void computeHemicubeFF();
        bool solveLights();
        bool solveMaterials();


        Mesh* mesh;

        GLuint fbo, fbo_rgb, fbo_z;

        int res;

        float** topHemicubeFF;
        float** sideHemicubeFF;

    public:
        class SampleData {
            public:
                Material radiosity;
                Material netIncoming;
                float fractionUnknown;
                std::vector<float> lightamount;
                int vertexid;
        };

        unsigned char** images;
        std::vector<SampleData> data;
        Material wallMaterial;
        Material floorMaterial;
        std::vector<Material> lights;

};

#endif
