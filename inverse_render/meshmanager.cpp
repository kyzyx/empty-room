#include "meshmanager.h"

#include <pcl/io/ply_io.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>
#include <boost/filesystem.hpp>

#include <functional>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace pcl;

#define SHM_MESHDATA_ID "SHM_MESHDATA_ID"
#define SAMPLE_SUFFIX "_SAMPLES"
// TODO: Proper locking mechanism

/*
 * Memory layout:
 * All channel mutexes
 * Sample mutex
 * Number of samples
 * Vertex Positions
 * Vertex Normals
 * Face indices
 * All labels
 */

// --------------------------------------------------------------
// Constructors and initialization
// --------------------------------------------------------------
MeshManager::MeshManager(int numvertices, int numfaces)
    : nvertices(numvertices), nfaces(numfaces)
{
    defaultinit("");
    initializeSharedMemory();
}
MeshManager::MeshManager(const string& meshfile)
    : nvertices(0), nfaces(0)
{
    readHeader(meshfile);
    defaultinit(meshfile);
    initializeSharedMemory();
}

void MeshManager::defaultinit(const string& meshfile) {
    string fullpath = boost::filesystem::canonical(boost::filesystem::path(meshfile)).string();
    hash<string> str_hash;
    stringstream ss;
    ss << setbase(16) << SHM_MESHDATA_ID << str_hash(fullpath);
    shmname = ss.str();
    shmsamplename = ss.str() + SAMPLE_SUFFIX;

    sampleinit = false;
}

void MeshManager::readHeader(const string& meshfile) {
    ifstream in(meshfile);
    string line;
    string s;
    getline(in, line);
    if (line == "ply") {
        getline(in, line);
        while (line != "end_header") {
            stringstream sin(line);
            sin >> s;
            if (s == "element") {
                sin >> s;
                if (s == "vertex") {
                    sin >> nvertices;
                } else if (s == "face") {
                    sin >> nfaces;
                }
            }
            getline(in, line);
        }
    } else {
        cerr << "Error parsing PLY header" << endl;
    }
    wsamples.resize(nvertices);
}

bool MeshManager::initializeR3Mesh() {
    m = new R3Mesh();
    if (!m) return false;
    for (int i = 0; i < nvertices; ++i) {
        m->CreateVertex(pos[i]);
    }
    for (int i = 0; i < nfaces; ++i) {
        m->CreateFace(m->Vertex(VertexOnFace(i,0)),
                      m->Vertex(VertexOnFace(i,1)),
                      m->Vertex(VertexOnFace(i,2)));
    }
    return true;
}

using namespace boost::interprocess;
bool MeshManager::initializeSharedMemory() {
    try {
        shared_memory_object shm(open_or_create, shmname.c_str(), read_write);
        shm.truncate(computeSize());
        mregion = mapped_region(shm, read_write);
    } catch(...) {
        return false;
    }

    char* s = static_cast<char*>(mregion.get_address());
    mutexes = (shmutex*)(s);
    s += (NUM_CHANNELS+1)*sizeof(shmutex);
    numsamples = (int*)(s);
    s += sizeof(int);
    pos = (R3Point*)(s);
    s += nvertices*sizeof(R3Point);
    norm = (R3Vector*)(s);
    s += nvertices*sizeof(R3Vector);
    faces = (int*)(s);
    s += nfaces*3*sizeof(int);
    for (int i = 0; i < NUM_CHANNELS; ++i) {
        labels[i] = (char*) s;
        s += nvertices;
    }
    return true;
}

bool MeshManager::initializeSharedSampleMemory() {
    if (*numsamples == 0) return false;
    try {
        shared_memory_object shm(open_or_create, shmsamplename.c_str(), read_write);
        int shmsize = nvertices*sizeof(int) + (*numsamples)*sizeof(Sample);
        shm.truncate(shmsize);
        sampleregion = mapped_region(shm, read_write);
    } catch(...) {
        return false;
    }

    char* s = static_cast<char*>(sampleregion.get_address());
    vertexsamples = (int*)(s);
    s += nvertices*sizeof(int);
    sampleptr = (Sample*)(s);
    samples.resize(nvertices,NULL);
    sampleinit = true;
    return true;
}

// --------------------------------------------------------------
// Utilities
// --------------------------------------------------------------
int MeshManager::computeSize() const {
    return (NUM_CHANNELS+1)*sizeof(shmutex)
         + sizeof(int)
         + nvertices*sizeof(R3Point)
         + nvertices*sizeof(R3Vector)
         + nfaces*3*sizeof(int)
         + NUM_CHANNELS*nvertices;
}

MeshManager::shmutex* MeshManager::getMutex(int t, int n) {
    return &mutexes[t];
}

// --------------------------------------------------------------
// Accessors
// --------------------------------------------------------------
R3Point MeshManager::VertexPosition(int n) const {
    return pos[n];
}
R3Vector MeshManager::VertexNormal(int n) const {
    return norm[n];
}
Eigen::Vector3f MeshManager::VertexPositionE(int n) const {
    return Eigen::Vector3f(pos[n].X(), pos[n].Y(), pos[n].Z());
}
Eigen::Vector3f MeshManager::VertexNormalE(int n) const {
    return Eigen::Vector3f(norm[n].X(), norm[n].Y(), norm[n].Z());
}
int MeshManager::VertexOnFace(int n, int i) const {
    return faces[3*n+i];
}
Sample MeshManager::getSample(int n, int i) const {
    if (!hasSamples()) return Sample();
    return samples[n][i];
}
int MeshManager::getVertexSampleCount(int n) const {
    if (!hasSamples()) return 0;
    return vertexsamples[n];
}
int MeshManager::getTotalSamples() {
    if (!hasSamples()) {
        return 0;
    }
    else {
        boost::interprocess::sharable_lock<shmutex> lock(*getMutex(NUM_CHANNELS, 0));
        return *numsamples;
    }
}

char MeshManager::getLabel(int n, int ch) const {
    //boost::interprocess::shareable_lock<shmutex> lock(*getMutex(ch,n));
    return labels[ch][n];
}
void MeshManager::setLabel(int n, char c, int ch) {
    //boost::interprocess::scoped_lock<shmutex> lock(*getMutex(ch, n));
    labels[ch][n] = c;
}
R3Mesh* MeshManager::getMesh() {
    if (!r3meshinitialized) {
        if (!initializeR3Mesh()) return NULL;
    }
    return m;
}

// --------------------------------------------------------------
// Sample Data Functions
// --------------------------------------------------------------
void MeshManager::addSample(int n, Sample s) {
    wsamples[n].push_back(s);
}
void MeshManager::commitSamples(bool finalcommit) {
    boost::interprocess::scoped_lock<shmutex> lock(*getMutex(NUM_CHANNELS, 0));
    if (finalcommit && *numsamples > 0) return;
    *numsamples = 0;
    for (int i = 0; i < nvertices; ++i) {
        *numsamples += wsamples[i].size();
    }
    initializeSharedSampleMemory();
    Sample* s = sampleptr;
    for (int i = 0; i < nvertices; ++i) {
        vertexsamples[i] = wsamples[i].size();
        samples[i] = s;
        for (int j = 0; j < vertexsamples[i]; ++j) {
            *s++ = wsamples[i][j];
        }
    }
    if (finalcommit) wsamples.clear();
}
bool MeshManager::loadSamples() {
    boost::interprocess::sharable_lock<shmutex> lock(*getMutex(NUM_CHANNELS, 0));
    if (*numsamples > 0) {
        if (!initializeSharedSampleMemory()) return false;
        Sample* s = sampleptr;
        for (int i = 0; i < nvertices; ++i) {
            samples[i] = s;
            s += vertexsamples[i];
        }
        wsamples.clear();
        return true;
    }
    return false;
}

Material MeshManager::getVertexColor(int n) const {
    Material m(0,0,0);
    double total = 0;
    for (int i = 0; i < vertexsamples[n]; ++i) {
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
void MeshManager::readSamplesFromFile(const std::string& samplesfile, cb_type cb) {
    ifstream in(samplesfile.c_str(), ifstream::binary);
    uint32_t n, sz;
    in.read((char*) &n, 4);
    for (int i = 0; i < n; ++i) {
        char c;
        in.read(&c, 1);
        setLabel(i, c, 0);
        in.read(&c, 1);
        setLabel(i, c, 1);
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
            addSample(i, s);
        }
        if (cb && (i+1) % (nvertices/100) == 0) cb((int) 80*(i/(float)nvertices));
    }
    commitSamples();
    if (cb) cb(100);
}
void MeshManager::writeSamplesToFile(const std::string& samplesfile, cb_type cb) {
    ofstream out(samplesfile.c_str(), ofstream::binary);
    uint32_t sz = nvertices;
    out.write((char*) &sz, 4);
    for (int i = 0; i < nvertices; ++i) {
        char c = getLabel(i,0);
        char t = getLabel(i,1);
        out.write(&c, 1);
        out.write(&t, 1);
        sz = getVertexSampleCount(i);
        out.write((char*) &sz, 4);
        for (int j = 0; j < sz; ++j) {
            Sample s = getSample(i,j);
            out.write((char*) &s.label, 1);
            out.write((char*) &s.r, sizeof(float));
            out.write((char*) &s.g, sizeof(float));
            out.write((char*) &s.b, sizeof(float));
            out.write((char*) &s.x, sizeof(float));
            out.write((char*) &s.y, sizeof(float));
            out.write((char*) &s.z, sizeof(float));
            out.write((char*) &s.confidence, sizeof(float));
            out.write((char*) &s.dA, sizeof(float));
        }
        if (cb && (i+1) % (nvertices/100) == 0) cb((int) 100*(i/(float)nvertices));
    }
    if (cb) cb(100);
}
