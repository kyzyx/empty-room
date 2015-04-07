#include "meshmanager.h"

#include <pcl/io/ply_io.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>

#include <functional>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace pcl;

#define SHM_MESHDATA_ID "SHM_MESHDATA_ID"
// TODO: Proper locking mechanism

/*
 * Memory layout:
 * All mutexes
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

void MeshManager::defaultinit(const string& meshfie) {
    hash<string> str_hash;
    stringstream ss;
    ss << setbase(16) << SHM_MESHDATA_ID << str_hash(meshfie);
    shmname = ss.str();
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
    s += NUM_CHANNELS*sizeof(shmutex);
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

// --------------------------------------------------------------
// Utilities
// --------------------------------------------------------------
int MeshManager::computeSize() const {
    return NUM_CHANNELS*sizeof(shmutex)
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
const R3Point MeshManager::VertexPosition(int n) const {
    return pos[n];
}
const R3Vector MeshManager::VertexNormal(int n) const {
    return norm[n];
}
const int MeshManager::VertexOnFace(int n, int i) const {
    return faces[3*n+i];
}
const char MeshManager::getLabel(int n, int ch) const {
    //boost::interprocess::shareable_lock<shmutex> lock(*getMutex(ch,n));
    return labels[ch][n];
}
void MeshManager::setLabel(int n, char c, int ch) {
    boost::interprocess::scoped_lock<shmutex> lock(*getMutex(ch, n));
    labels[ch][n] = c;
}
R3Mesh* MeshManager::getMesh() {
    if (!r3meshinitialized) {
        if (!initializeR3Mesh()) return NULL;
    }
    return m;
}
