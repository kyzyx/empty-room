#ifndef _MESH_MANAGER_H
#define _MESH_MANAGER_H

#include "R3Shapes/R3Shapes.h"
#include <string>
#include <vector>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_upgradable_mutex.hpp>

class MeshManager {
    public:
        typedef boost::interprocess::interprocess_upgradable_mutex shmutex;

        MeshManager(int numvertices, int numfaces);
        MeshManager(const std::string& meshfile);

        const R3Point VertexPosition(int n) const;
        const R3Vector VertexNormal(int n) const;
        const int VertexOnFace(int n, int i) const;
        const char getLabel(int n, int ch=0) const;
        void setLabel(int n, char c, int ch=0);

        R3Mesh* getMesh();

        int size() const { return nvertices; }
        int NVertices() const { return nvertices; }
        int NFaces() const { return nfaces; }

        enum {
            LABEL_CHANNEL,
            TYPE_CHANNEL,
            DATA_CHANNEL,
            NUM_CHANNELS
        };

    protected:
        MeshManager() { ; }

        void readHeader(const std::string& meshfile);
        void defaultinit(const std::string& meshfile);
        bool initializeSharedMemory();
        int computeSize() const;

        shmutex* getMutex(int t, int n);

        bool initializeR3Mesh();
        bool r3meshinitialized;
        R3Mesh* m;

        int nvertices, nfaces;
        R3Point* pos;
        R3Vector* norm;
        int* faces;
        char* labels[NUM_CHANNELS];

        shmutex* mutexes;
        std::string shmname;

        boost::interprocess::mapped_region mregion;
};
#endif
