#include "mesh.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace pcl;

Mesh::~Mesh() {
    delete mesh;
    delete searchtree;
}
Mesh::Mesh(PolygonMesh::Ptr m) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    fromPCLPointCloud2(m->cloud, *cloud);

    mesh = new R3Mesh();

    // Copy all vertices
    PointCloud<PointXYZ>::iterator it;
    for (it = cloud->begin(); it != cloud->end(); ++it) {
        mesh->CreateVertex(R3Point((*it).x, (*it).y, (*it).z));
    }
    // Copy all faces, assuming triangular
    for (int i = 0; i < m->polygons.size(); ++i) {
        if (m->polygons[i].vertices.size() != 3) break;
        mesh->CreateFace(mesh->Vertex(m->polygons[i].vertices[0]), mesh->Vertex(m->polygons[i].vertices[1]), mesh->Vertex(m->polygons[i].vertices[2]));
    }

    searchtree = new R3MeshSearchTree(mesh);

    labels.resize(mesh->NVertices(), 0);
    samples.resize(mesh->NVertices());
}

void Mesh::addSample(int n, Sample s) {
    samples[n].push_back(s);
}
void Mesh::addLabel(int n, int label) {
    labels[n] = label;
}
