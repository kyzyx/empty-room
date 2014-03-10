#ifndef _MESH_H
#define _MESH_H

#include "RNBasics/RNBasics.h"
#include "R3Shapes/R3Shapes.h"
#include <pcl/PolygonMesh.h>
#include <vector>

/**
 * Stores a single sample of the outgoing radiance from a surface
 * at a particular direction.
 */
class Sample {
    public:
        unsigned char r;
        unsigned char g;
        unsigned char b;
        float x;
        float y;
        float z;
        float dA;
        // Larger solid angle implies less accurate colors, since more of
        // the pixel actually comes from other vertices
        // Larger form factor implies more accuracy - the projection of the
        // mesh element on the pixel is greater and therefore is more
};

class Mesh {
    public:
        Mesh(pcl::PolygonMesh::Ptr mesh);
        ~Mesh();
        void addSample(int n, Sample s);
        void addLabel(int n, int label);

        R3Mesh* getMesh() { return mesh; }
        R3MeshSearchTree* getSearchTree() { return searchtree; }

        std::vector<std::vector<Sample> > samples;
        std::vector<int> labels;
    private:
        Mesh() {;}

        R3Mesh* mesh;
        R3MeshSearchTree* searchtree;
};
#endif
