#ifndef _MESH_H
#define _MESH_H

#include <GL/gl.h>
#include "RNBasics/RNBasics.h"
#include "R3Shapes/R3Shapes.h"
#include <pcl/PolygonMesh.h>
#include <vector>
#include <string>
#define MAX_LIGHTS 128

/**
 * Stores a single sample of the outgoing radiance from a surface
 * at a particular direction.
 */
class Sample {
    public:
        unsigned char label;
        float r;
        float g;
        float b;
        float x;
        float y;
        float z;
        float confidence;
        float dA;
        // Larger solid angle implies less accurate colors, since more of
        // the pixel actually comes from other vertices
        // Larger form factor implies more accuracy - the projection of the
        // mesh element on the pixel is greater and therefore is more
};

class Mesh {
    public:
        Mesh(pcl::PolygonMesh::Ptr mesh, bool initOGL=false);
        ~Mesh();
        void addSample(int n, Sample s);

        void writeColoredMesh(std::string filename, double scalefactor=10);
        void writeSamples(std::string filename);
        int readSamples(std::string filename);

        R3Mesh* getMesh() { return mesh; }
        const R3Mesh* getMesh() const { return mesh; }
        R3MeshSearchTree* getSearchTree() { return searchtree; }
        void renderOGL(bool light=false) const;
        void computeColorsOGL();

        std::vector<std::vector<Sample> > samples;
        std::vector<char> labels;
        std::vector<char> types;
    private:
        Mesh() {;}

        GLuint vbo, ibo, cbo;

        R3Mesh* mesh;
        R3MeshSearchTree* searchtree;
};
#endif
