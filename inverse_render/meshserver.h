#ifndef _MESH_SERVER_H
#define _MESH_SERVER_H
#include "meshmanager.h"
#include <pcl/PolygonMesh.h>

class MeshServer : public MeshManager {
    public:
        MeshServer(const std::string& meshfile,
                bool ccw=false,
                void (*progress_callback)(int)=NULL);
        MeshServer(pcl::PolygonMesh::ConstPtr mesh,
                bool ccw=false,
                void (*progress_callback)(int)=NULL);
        ~MeshServer();

    protected:
        bool loadMesh(pcl::PolygonMesh::ConstPtr mesh, bool ccw=false);
        void (*progress_cb)(int);
};
#endif
