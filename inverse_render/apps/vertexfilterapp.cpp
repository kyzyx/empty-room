#include "invrenderapp.h"
#include "featurefinder.h"
#include <pcl/console/parse.h>

using namespace std;

class VertexFilterApp : public InvrenderApp {
    public:
        VertexFilterApp()
            : idx(-1), lbl(-1)
        {
            float m = numeric_limits<float>::lowest();
            for (int i = 0; i < 3; i++) minp[i] = m;
            m = numeric_limits<float>::max();
            for (int i = 0; i < 3; i++) maxp[i] = m;
        }
        virtual int run() {
            vector<int> indices;
            FloorplanHelper fph;
            fph.loadFromRoomModel(room);
            FeatureFinder ff;
            if (lbl >= 0)
                ff.condition.setLabel(lbl);
            if (idx >= 0)
                ff.condition.setWallIndex(idx);
            for (int i = 0; i < 3; i++) {
                ff.condition.setMax(i, maxp[i]);
                ff.condition.setMin(i, minp[i]);
            }
            ff.computeIndices(fph, mmgr, indices);
            for (int i = 0; i < indices.size(); i++) {
                cout << ">>data:" << indices[i] << endl;
            }
            cout << ">>done" << endl;
            return 0;
        }
    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (!room) return 0;
            if (pcl::console::find_argument(argc, argv, "-wallidx") >= 0) {
                pcl::console::parse_argument(argc, argv, "-wallidx", idx);
            }
            if (pcl::console::find_argument(argc, argv, "-label") >= 0) {
                pcl::console::parse_argument(argc, argv, "-label", lbl);
            }
            if (pcl::console::find_argument(argc, argv, "-minx") >= 0) {
                pcl::console::parse_argument(argc, argv, "-minx", minp[0]);
            }
            if (pcl::console::find_argument(argc, argv, "-miny") >= 0) {
                pcl::console::parse_argument(argc, argv, "-miny", minp[1]);
            }
            if (pcl::console::find_argument(argc, argv, "-minz") >= 0) {
                pcl::console::parse_argument(argc, argv, "-minz", minp[2]);
            }
            if (pcl::console::find_argument(argc, argv, "-maxx") >= 0) {
                pcl::console::parse_argument(argc, argv, "-maxx", maxp[0]);
            }
            if (pcl::console::find_argument(argc, argv, "-maxy") >= 0) {
                pcl::console::parse_argument(argc, argv, "-maxy", maxp[1]);
            }
            if (pcl::console::find_argument(argc, argv, "-maxz") >= 0) {
                pcl::console::parse_argument(argc, argv, "-maxz", maxp[2]);
            }
            return 1;
        }

        float minp[3];
        float maxp[3];
        int idx, lbl;
};

int main(int argc, char** argv) {
    VertexFilterApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
