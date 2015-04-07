#ifndef _MESH_SERVER_H
#define _MESH_SERVER_H
#include "meshmanager.h"
#include <pcl/PolygonMesh.h>

class MeshServer : public MeshManager {
    public:
        MeshServer(const std::string& meshfile, bool ccw=false);
        MeshServer(pcl::PolygonMesh::ConstPtr mesh, bool ccw=false);
        ~MeshServer();

    protected:
        bool loadMesh(pcl::PolygonMesh::ConstPtr mesh, bool ccw=false);
};
#endif
