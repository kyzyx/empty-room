#include "invrenderapp.h"
#include <pcl/console/parse.h>

using namespace std;

class SubdivApp : public InvrenderApp {
    public:
        SubdivApp() : maxlength(0.02) {}
    virtual int run() {
        R3Mesh m;
        m.ReadPlyFile(infile.c_str());
        m.SubdivideLongEdges(maxlength);
        m.WritePlyFile(outfile.c_str());
    }

    protected:
        virtual int _parseargs(int argc, char** argv) {
            if (pcl::console::find_argument(argc, argv, "-out") >= 0) {
                pcl::console::parse_argument(argc, argv, "-out", outfile);
            }
            if (pcl::console::find_argument(argc, argv, "-in") >= 0) {
                pcl::console::parse_argument(argc, argv, "-in", infile);
            }
            if (pcl::console::find_argument(argc, argv, "-maxedgelength") >= 0) {
                pcl::console::parse_argument(argc, argv, "-maxedgelength", maxlength);
            }
        }
        string infile;
        string outfile;
        double maxlength;
};

int main(int argc, char** argv) {
    SubdivApp app;
    if (!app.parseargs(argc, argv)) return 1;
    return app.run();
}
