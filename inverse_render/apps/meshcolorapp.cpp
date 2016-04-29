#include "invrenderapp.h"
#include <pcl/console/parse.h>
#include <queue>

using namespace std;

class MeshColorApp : public InvrenderApp {
    public:
        MeshColorApp()
        {
        }
        virtual int run() {
            mmgr->loadSamples();
            R3Mesh* m = mmgr->getMesh();
            vector<Material> colors;
            queue<int> q;
            for (int i = 0; i < mmgr->size(); i++) {
                colors.push_back(mmgr->getMedianVertexColor(i));
            }

            for (int i = 0; i < mmgr->size(); i++) {
                if (colors[i].r == 0 && colors[i].g == 0 && colors[i].b == 0) {
                    q.push(i);
                }
            }
            while (!q.empty()) {
                int vid = q.front(); q.pop();
                R3MeshVertex* v = m->Vertex(vid);
                Material mat(0,0,0);
                int count = 0;
                for (int i = 0; i < m->VertexValence(v); i++) {
                    int j = m->VertexID(m->VertexOnVertex(v, i));
                    if (colors[j].r != 0 && colors[j].g != 0 && colors[j].b != 0) {
                        mat += colors[j];
                        count++;
                    }
                }
                if (count) {
                    mat.r /= count;
                    mat.g /= count;
                    mat.b /= count;
                    colors[vid] = mat;
                }
            }
            mmgr->writePlyMesh(filename, colors, 0.5, 1/2.2);
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (pcl::console::find_argument(argc, argv, "-filename") >= 0) {
                pcl::console::parse_argument(argc, argv, "-filename", filename);
            }
            return 1;
        }
        string filename;
};

int main(int argc, char** argv) {
    MeshColorApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
