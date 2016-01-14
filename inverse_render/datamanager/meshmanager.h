#ifndef _MESH_MANAGER_H
#define _MESH_MANAGER_H

#include "R3Shapes/R3Shapes.h"
#include "material.h"
#include <string>
#include <vector>
#include <Eigen/Eigen>

#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>
#include <boost/function.hpp>
/**
 * Stores a single sample of the outgoing radiance from a surface
 * at a particular direction.
 */
class Sample {
    public:
        float r;
        float g;
        float b;
        float x;
        float y;
        float z;
        float confidence;
        float dA;
        // Larger solid angle implies less accurate colors, since more of
        // the pixel actually comes from other vertices
        // Larger form factor implies more accuracy - the projection of the
        // mesh element on the pixel is greater and therefore is more
        uint16_t label;
        int16_t id;
};

class MeshManager {
    public:
        typedef boost::interprocess::interprocess_sharable_mutex shmutex;
        typedef boost::function<void(int)> cb_type;

        MeshManager(int numvertices, int numfaces);
        MeshManager(const std::string& meshfile);

        // Data accessors
        R3Point VertexPosition(int n) const;
        R3Vector VertexNormal(int n) const;
        Eigen::Vector3f VertexPositionE(int n) const;
        Eigen::Vector3f VertexNormalE(int n) const;
        int VertexOnFace(int n, int i) const;
        char getLabel(int n, int ch=0) const;
        Sample getSample(int n, int i) const;
        Material getVertexColor(int n) const;
        int getVertexSampleCount(int n) const;
        int getTotalSamples();

        // Label i/o
        void setLabel(int n, char c, int ch=0);

        R3Mesh* getMesh();

        void writePlyMesh(const std::string& filename, double scalefactor=1, double gamma=1);

        // Size accessors
        int size() const { return nvertices; }
        int NVertices() const { return nvertices; }
        int NFaces() const { return nfaces; }
        bool hasSamples() const { return sampleinit; }

        // Sample setup functions
        void addSample(int n, Sample s);
        void commitSamples(bool finalcommit=true);
        bool loadSamples();

        // Sample I/O
        void readSamplesFromFile(const std::string& samplesfile, int flags=-1, cb_type cb=NULL);
        void writeSamplesToFile(const std::string& samplesfile, cb_type cb=NULL);
        // Vertex info I/O
        void readLabelsFromFile(const std::string& labelsfile, int flags=-1, cb_type cb=NULL);
        void writeLabelsToFile(const std::string& labelsfile, cb_type cb=NULL);

        shmutex* getMutex(int t, int n);

        enum {
            LABEL_CHANNEL,
            TYPE_CHANNEL,
            DATA_CHANNEL,
            NUM_CHANNELS
        };
        enum {
            READ_SAMPLES_ONLY=0,
            READ_LABEL_CHANNEL=1,
            READ_TYPE_CHANNEL=2,
            READ_DATA_CHANNEL=4,
            READ_ALL_CHANNELS=7,
        };

    protected:
        MeshManager() { ; }

        void readHeader(const std::string& meshfile);
        void defaultinit(const std::string& meshfile);
        bool initializeSharedMemory();
        int computeSize() const;
        bool initializeSharedSampleMemory();


        // R3Mesh utilities
        bool initializeR3Mesh();
        bool r3meshinitialized;
        R3Mesh* m;

        // Geometry data
        int nvertices, nfaces;
        R3Point* pos;
        R3Vector* norm;
        int* faces;
        // Auxiliary Data
        char* labels[NUM_CHANNELS];
        // Sample data
        int* numsamples;
        int* vertexsamples;
        Sample* sampleptr;
        std::vector<Sample*> samples;
        std::vector<std::vector<Sample> > wsamples;
        bool sampleinit;

        shmutex* mutexes;
        std::string shmname;
        std::string shmsamplename;

        boost::interprocess::mapped_region mregion;
        boost::interprocess::mapped_region sampleregion;
};
#endif
