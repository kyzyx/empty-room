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

    float* vertices;
    if (initOGL) {
        vertices = new float[3*3*m->polygons.size()];
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
    }

    // Copy all vertices
    PointCloud<PointXYZ>::iterator it;
    for (it = cloud->begin(); it != cloud->end(); ++it) {
        mesh->CreateVertex(R3Point((*it).x, (*it).y, (*it).z));
    }

    int z = 0;
    int v = 0;
    // Copy all faces, assuming triangular
    for (int i = 0; i < m->polygons.size(); ++i) {
        if (m->polygons[i].vertices.size() != 3) break;
        mesh->CreateFace(mesh->Vertex(m->polygons[i].vertices[0]), mesh->Vertex(m->polygons[i].vertices[1]), mesh->Vertex(m->polygons[i].vertices[2]));
        if (initOGL) {
            R3Point p = mesh->VertexPosition(mesh->Vertex(m->polygons[i].vertices[0]));
            vertices[v++] = p[0];
            vertices[v++] = p[1];
            vertices[v++] = p[2];
            p = mesh->VertexPosition(mesh->Vertex(m->polygons[i].vertices[1]));
            vertices[v++] = p[0];
            vertices[v++] = p[1];
            vertices[v++] = p[2];
            p = mesh->VertexPosition(mesh->Vertex(m->polygons[i].vertices[2]));
            vertices[v++] = p[0];
            vertices[v++] = p[1];
            vertices[v++] = p[2];
        }
    }
    if (initOGL) {
        glBufferData(GL_ARRAY_BUFFER,
                3*3*m->polygons.size()*sizeof(float),
                vertices, GL_STATIC_DRAW);
        delete [] vertices;
    }

    searchtree = new R3MeshSearchTree(mesh);

    labels.resize(mesh->NVertices(), 0);
    types.resize(mesh->NVertices(), 0);
    samples.resize(mesh->NVertices());
}

void Mesh::addSample(int n, Sample s) {
    samples[n].push_back(s);
}

void Mesh::writeColoredMesh(std::string filename) {
    ofstream out(filename);
    int n = mesh->NVertices();
    int f = mesh->NFaces();
    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << n << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "property uchar red" << endl;
    out << "property uchar green" << endl;
    out << "property uchar blue" << endl;
    out << "element face " << f << endl;
    out << "property list uchar int vertex_indices" << endl;
    out << "end_header" << endl;
    for (int i = 0; i < n; ++i) {
        R3Point p = mesh->VertexPosition(mesh->Vertex(i));
        out << p[0] << " " << p[1] << " " << p[2] << " ";
        Material m = getVertexColor(i);
        m *= 195;
        out << min((int)m.r,255) << " " << min((int)m.g,255) << " " << min((int)m.b,255) << endl;
    }
    for (int i = 0; i < f; ++i) {
        out << "3";
        for (int j = 0; j < 3; ++j) {
            int vid = mesh->VertexID(mesh->VertexOnFace(mesh->Face(i), j));
            out << " " << vid;
        }
        out << endl;
    }
}

void Mesh::writeSamples(string filename) {
    ofstream out(filename.c_str(), ofstream::binary);
    uint32_t sz = samples.size();
    out.write((char*) &sz, 4);
    for (int i = 0; i < samples.size(); ++i) {
        char c = labels[i];
        char t = types[i];
        out.write(&c, 1);
        out.write(&t, 1);
        sz = samples[i].size();
        out.write((char*) &sz, 4);
        for (int j = 0; j < samples[i].size(); ++j) {
            out.write((char*) &samples[i][j].label, 1);
            out.write((char*) &samples[i][j].r, sizeof(float));
            out.write((char*) &samples[i][j].g, sizeof(float));
            out.write((char*) &samples[i][j].b, sizeof(float));
            out.write((char*) &samples[i][j].x, sizeof(float));
            out.write((char*) &samples[i][j].y, sizeof(float));
            out.write((char*) &samples[i][j].z, sizeof(float));
            out.write((char*) &samples[i][j].confidence, sizeof(float));
            out.write((char*) &samples[i][j].dA, sizeof(float));
        }
    }
}
int Mesh::readSamples(string filename) {
    vector<bool> islight(MAX_LIGHTS,false);
    int numlights = 0;
    ifstream in(filename.c_str(), ifstream::binary);
    uint32_t sz;
    in.read((char*) &sz, 4);
    samples.resize(sz);
    labels.resize(sz);
    types.resize(sz);
    for (int i = 0; i < samples.size(); ++i) {
        char c;
        in.read(&c, 1);
        labels[i] = c;
        if (c && !islight[c]) {
            islight[c] = true;
            ++numlights;
        }
        in.read(&c, 1);
        types[i] = c;
        in.read((char*) &sz, 4);
        for (int j = 0; j < sz; ++j) {
            Sample s;
            in.read((char*) &s.label, 1);
            in.read((char*) &s.r, sizeof(float));
            in.read((char*) &s.g, sizeof(float));
            in.read((char*) &s.b, sizeof(float));
            in.read((char*) &s.x, sizeof(float));
            in.read((char*) &s.y, sizeof(float));
            in.read((char*) &s.z, sizeof(float));
            in.read((char*) &s.confidence, sizeof(float));
            in.read((char*) &s.dA, sizeof(float));
            samples[i].push_back(s);
        }
    }
    return numlights;
}

void Mesh::renderOGL(bool light) const {
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
    glDrawArrays(GL_TRIANGLES, 0, mesh->NFaces()*3);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}

void Mesh::computeColorsOGL() {
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    float* vertices = new float[6*3*mesh->NFaces()];
    memset(vertices, 0, sizeof(float)*6*3*mesh->NFaces());
    for (int i = 0; i < mesh->NFaces(); ++i) {
        // If any vertex has no samples, discard this face
        bool valid = true;
        // If all vertices are the same light vertices, this is a light face
        int light = 0;
        for (int j = 0; j < 3; ++j) {
            int n = mesh->VertexID(mesh->VertexOnFace(mesh->Face(i), j));
            int ind = 6*(3*i + j);
            vertices[ind+1] = types[n]/128.;
            vertices[ind+2] = 1;
            if (samples[n].size() == 0) {
                valid = false;
            }
            if (light == 0 && labels[n] > 0) light = labels[n];
            else if (light > 0 && labels[n] == 0) light = -1; // Half light
            else if (light > 0 && labels[n] != light) light = -2; // Differing lights, should never happen
        }
        if (!valid) continue;
        for (int j = 0; j < 3; ++j) {
            int ind = 6*(3*i + j);
            if (light > 0) {
                vertices[ind+0] = light/(float)MAX_LIGHTS;
                vertices[ind+2] = 0;
            } else if (light != -1) {
                vertices[ind+2] = 0;
                int n = mesh->VertexID(mesh->VertexOnFace(mesh->Face(i), j));
                Material m = getVertexColor(n);
                vertices[ind+3] = m.r;
                vertices[ind+4] = m.g;
                vertices[ind+5] = m.b;
            }
        }
    }
    glBufferData(GL_ARRAY_BUFFER, 6*3*mesh->NFaces()*sizeof(float),
            vertices, GL_STATIC_DRAW);
    delete [] vertices;
}
Material Mesh::getVertexColor(int n) const {
    Material m(0,0,0);
    double total = 0;
    for (int i = 0; i < samples[n].size(); ++i) {
        const Sample& s = samples[n][i];
        if (s.confidence > 0) {
            m += Material(s.r,s.g,s.b)*abs(s.dA)*s.confidence;
            total += s.dA*s.confidence;
        }
    }
    if (total > 0) {
        m /= total;
    }
    return m;
}
