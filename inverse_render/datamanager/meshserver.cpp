#include "meshserver.h"

#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>

#include <iostream>
#include <fstream>

#include <boost/interprocess/shared_memory_object.hpp>

using namespace std;
using namespace pcl;

// --------------------------------------------------------------
// Constructors and initialization
// --------------------------------------------------------------
using namespace boost::interprocess;
MeshServer::MeshServer(const string& meshfile, bool ccw, cb_type cb)
    : progress_cb(cb)
{
    readHeader(meshfile);
    defaultinit(meshfile);
    shared_memory_object::remove(shmname.c_str());
    initializeSharedMemory();
    for (int i = 0; i < NUM_CHANNELS+1; ++i) {
        new (&(mutexes[i])) shmutex();
    }
    if (progress_cb) progress_cb(5);

    PolygonMesh::Ptr mesh(new PolygonMesh());
    io::loadPolygonFile(meshfile.c_str(), *mesh);
    if (progress_cb) progress_cb(33);
    loadMesh(mesh, ccw);
}
MeshServer::MeshServer(PolygonMesh::ConstPtr mesh, bool ccw, cb_type cb)
    : progress_cb(cb)
{
    nfaces = mesh->polygons.size();
    nvertices = mesh->cloud.width * mesh->cloud.height;
    wsamples.resize(nvertices);
    if (progress_cb) progress_cb(2);

    defaultinit(boost::lexical_cast<string>(mesh->header.stamp));
    shared_memory_object::remove(shmname.c_str());
    shared_memory_object::remove(shmsamplename.c_str());
    initializeSharedMemory();
    for (int i = 0; i < NUM_CHANNELS+1; ++i) {
        new (&(mutexes[i])) shmutex();
    }
    if (progress_cb) progress_cb(33);

    loadMesh(mesh, ccw);
}

MeshServer::~MeshServer() {
    shared_memory_object::remove(shmname.c_str());
    shared_memory_object::remove(shmsamplename.c_str());
}

bool MeshServer::loadMesh(PolygonMesh::ConstPtr mesh, bool ccw) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
    fromPCLPointCloud2(mesh->cloud, *cloud);
    if (progress_cb) progress_cb(50);

    for (int i = 0; i < nvertices; ++i) {
        pos[i] = R3Point(cloud->points[i].x,
                         cloud->points[i].y,
                         cloud->points[i].z);
    }
    if (progress_cb) progress_cb(70);

    for (int i = 0; i < nfaces; ++i) {
        int* f = faces+3*i;
        for (int j = 0; j < 3; ++j) {
            f[j] = mesh->polygons[i].vertices[j];
        }

        // Face normal
        R3Vector a = pos[f[0]] - pos[f[1]];
        R3Vector b = pos[f[2]] - pos[f[1]];
        R3Vector c = a%b;
        if (ccw) c = -c;

        // Update vertex normals
        for (int j = 0; j < 3; ++j) {
            norm[f[j]] += c;
        }
    }
    if (progress_cb) progress_cb(90);

    // Normalize normals
    for (int i = 0; i < nvertices; ++i) {
        norm[i].Normalize();
    }
    return true;
}
