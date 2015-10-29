#include "invrenderapp.h"
#include "exposuresolver.h"
#include <iostream>

using namespace std;

class ExposureSolverApp : public InvrenderApp {
    public:
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
            double* exposures = new double[3*n];
            for (int i = 0; i < 3*n; i++) exposures[i] = 1;
            double* radiances = new double[indices.size()];
            for (int i = 0; i < indices.size(); i++) radiances[i] = 50;
            cerr << "Computing " << indices.size() << " vertex radiances ";
            cerr << "with " << n << " images" << endl;
            solveExposure(mmgr, n, exposures, radiances, indices);
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
        }
};

int main(int argc, char** argv) {
    ExposureSolverApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
