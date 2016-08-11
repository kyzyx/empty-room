#include "invrenderapp.h"
#include "exposuresolver.h"
#include <iostream>
#include <pcl/console/parse.h>

using namespace std;

class ExposureSolverApp : public InvrenderApp {
    public:
        ExposureSolverApp() : subsample(0), scale(-1){}
        virtual int run() {
            vector<int> indices;
            int n = 0;
            mmgr->loadSamples();
            for (int i = 0; i < mmgr->NVertices(); i++) {
                if (mmgr->getVertexSampleCount(i) > 0) {
                    indices.push_back(i);
                    for (int j = 0; j < mmgr->getVertexSampleCount(i); j++) {
                        n = max(n, (int) mmgr->getSample(i, j).id);
                    }
                }
            }
            n++;
            if (subsample > 1) {
                cout << "Subsampling " << subsample << endl;
                random_shuffle(indices.begin(), indices.end());
                indices.resize(indices.size()/subsample);
            }

            double* exposures = new double[3*n];
            for (int i = 0; i < 3*n; i++) exposures[i] = 1;
            double* radiances = new double[indices.size()];
            for (int i = 0; i < indices.size(); i++) radiances[i] = 50;
            cerr << "Computing " << indices.size() << " vertex radiances ";
            cerr << "with " << n << " images" << endl;
            if (scale > 0) {
                solveExposure(mmgr, n, exposures, radiances, indices, LOSS_HUBER, scale);
            } else {
                solveExposure(mmgr, n, exposures, radiances, indices);
            }
            for (int i = 0; i < n; i++) {
                for (int ch = 0; ch < 3; ch++) {
                    if (ch) cout << " ";
                    cout << 1./exposures[i+ch*n];
                }
                cout << endl;
            }
        }
    protected:
        virtual void printargs() {
        }
        virtual int _parseargs(int argc, char** argv) {
            if (!mmgr) return 0;
            if (pcl::console::find_argument(argc, argv, "-subsample") >= 0) {
                pcl::console::parse_argument(argc, argv, "-subsample", subsample);
            }
            if (pcl::console::find_argument(argc, argv, "-scale") >= 0) {
                pcl::console::parse_argument(argc, argv, "-scale", scale);
            }
            return 1;
        }
        float subsample;
        float scale;
};

int main(int argc, char** argv) {
    ExposureSolverApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
