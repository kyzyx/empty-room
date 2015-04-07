#ifndef _MESH_SERVER_H
#define _MESH_SERVER_H
#include "meshmanager.h"

class MeshServer : public MeshManager {
    public:
        MeshServer(const std::string& meshfile, bool ccw=false);
        ~MeshServer();

    protected:
        bool loadMesh(const std::string& meshfile, bool ccw=false);
};
#endif
