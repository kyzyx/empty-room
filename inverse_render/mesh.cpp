#include <GL/glew.h>
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
Mesh::Mesh(PolygonMesh::Ptr m, bool initOGL) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    fromPCLPointCloud2(m->cloud, *cloud);

    mesh = new R3Mesh();

    GLuint* indices;
    float* vertices;
    if (initOGL) {
        indices = new GLuint[3*m->polygons.size()];
        vertices = new float[3*cloud->size()];
        cout << "Finished allocating memory..." << endl;
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glGenBuffers(1, &ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        cout << "Finished allocating graphics memory..." << endl;
    }

    // Copy all vertices
    int z = 0;
    PointCloud<PointXYZ>::iterator it;
    for (it = cloud->begin(); it != cloud->end(); ++it) {
        mesh->CreateVertex(R3Point((*it).x, (*it).y, (*it).z));
        if (initOGL) {
            vertices[z++] = it->x;
            vertices[z++] = it->y;
            vertices[z++] = it->z;
        }
    }


    z = 0;
    // Copy all faces, assuming triangular
    for (int i = 0; i < m->polygons.size(); ++i) {
        if (m->polygons[i].vertices.size() != 3) break;
        mesh->CreateFace(mesh->Vertex(m->polygons[i].vertices[0]), mesh->Vertex(m->polygons[i].vertices[1]), mesh->Vertex(m->polygons[i].vertices[2]));
        if (initOGL) {
            indices[z++] = m->polygons[i].vertices[0];
            indices[z++] = m->polygons[i].vertices[1];
            indices[z++] = m->polygons[i].vertices[2];
        }
    }
    if (initOGL) {
        glBufferData(GL_ARRAY_BUFFER,
                3*cloud->size()*sizeof(float),
                vertices, GL_STATIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                3*m->polygons.size()*sizeof(GLuint),
                indices, GL_STATIC_DRAW);
        delete [] indices;
        delete [] vertices;
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

void Mesh::renderOGL(bool light) {
    glDepthMask(true);
    glClearColor(0.,0.,0.,0.);
    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glColorPointer(3, GL_FLOAT, 6*sizeof(float), (void*) (light?3*sizeof(float):0));
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, 3*sizeof(float), 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glDrawElements(GL_TRIANGLES, mesh->NFaces()*3, GL_UNSIGNED_INT,(void*)0);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}
void Mesh::computeColorsOGL() {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    float* vertices = new float[6*mesh->NVertices()];
    for (int i = 0; i < mesh->NVertices(); ++i) {
        const R3Point& p = mesh->VertexPosition(mesh->Vertex(i));
        for (int j = 0; j < 6; ++j) vertices[6*i + j] = 0;
        float total = 0;
        float ltotal = 0;
        if (samples[i].size() == 0) {
            vertices[6*i+0] = 0;
            vertices[6*i+1] = 0;
            vertices[6*i+2] = 1;
        } else {
            for (int j = 0; j < samples[i].size(); ++j) {
                float s = abs(samples[i][j].dA);
                if (samples[i][j].label > 0) {
                    vertices[6*i+0] += samples[i][j].r*s;
                    vertices[6*i+1] += samples[i][j].g*s;
                    vertices[6*i+2] += samples[i][j].b*s;
                    ltotal += s;
                } else {
                    vertices[6*i+3] += samples[i][j].r*s;
                    vertices[6*i+4] += samples[i][j].g*s;
                    vertices[6*i+5] += samples[i][j].b*s;
                    total += s;
                }
            }
            vertices[6*i+0] /= ltotal*255;
            vertices[6*i+1] /= ltotal*255;
            vertices[6*i+2] /= ltotal*255;
            vertices[6*i+3] /= total*255;
            vertices[6*i+4] /= total*255;
            vertices[6*i+5] /= total*255;
        }
    }
    glBufferData(GL_ARRAY_BUFFER, 6*mesh->NVertices()*sizeof(float),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
}
