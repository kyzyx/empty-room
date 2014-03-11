#include "mesh.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <fstream>

using namespace pcl;
using namespace std;

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
void Mesh::writeSamples(string filename) {
    ofstream out(filename.c_str(), ofstream::binary);
    uint32_t sz = samples.size();
    out.write((char*) &sz, 4);
    for (int i = 0; i < samples.size(); ++i) {
        char c = labels[i];
        out.write(&c, 1);
        sz = samples[i].size();
        out.write((char*) &sz, 4);
        for (int j = 0; j < samples[i].size(); ++j) {
            out.write((char*) &samples[i][j].label, 1);
            out.write((char*) &samples[i][j].r, 1);
            out.write((char*) &samples[i][j].g, 1);
            out.write((char*) &samples[i][j].b, 1);
            out.write((char*) &samples[i][j].x, sizeof(float));
            out.write((char*) &samples[i][j].y, sizeof(float));
            out.write((char*) &samples[i][j].z, sizeof(float));
            out.write((char*) &samples[i][j].dA, sizeof(float));
        }
    }
}
void Mesh::readSamples(string filename) {
    ifstream in(filename.c_str(), ifstream::binary);
    uint32_t sz;
    in.read((char*) &sz, 4);
    samples.resize(sz);
    labels.resize(sz);
    for (int i = 0; i < samples.size(); ++i) {
        char c;
        in.read(&c, 1);
        labels[i] = c;
        in.read((char*) &sz, 4);
        for (int j = 0; j < sz; ++j) {
            Sample s;
            in.read((char*) &s.label, 1);
            in.read((char*) &s.r, 1);
            in.read((char*) &s.g, 1);
            in.read((char*) &s.b, 1);
            in.read((char*) &s.x, sizeof(float));
            in.read((char*) &s.y, sizeof(float));
            in.read((char*) &s.z, sizeof(float));
            in.read((char*) &s.dA, sizeof(float));
            samples[i].push_back(s);
        }
    }
}
