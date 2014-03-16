#include "rerender.h"
#include <fstream>

using namespace std;

void outputRadianceFile(string filename, WallFinder& wf, Mesh& m, InverseRender& ir) {
    ofstream out(filename);
    // output wall material
    out << "void plastic wallmat" << endl << 0 << endl << 0 << endl << 5 << " ";
    out << ir.wallMaterial(0) << " " << ir.wallMaterial(1) << " " << ir.wallMaterial(2);
    out << " 0 0" << endl << endl;

    // Output light materials
    for (int i = 0; i < ir.lights.size(); ++i) {
        out << "void light l" << i << endl << 0 << endl << 0 << endl << 3 << " ";
        out << ir.lights[i](0) << " " << ir.lights[i](1) << " " << ir.lights[i](2) << endl << endl;
    }

    // Output light triangles;
    for (int i = 0; i < m.getMesh()->NFaces(); ++i) {
    }

    double hi = wf.ceilplane;
    double lo = wf.floorplane;
    double eps = 0.00001;

    out << "l0 polygon light0\n0 0 12\n" \
        "1.1 1.6 6  1.7 1.6 6 1.7 1.0 6 1.1 1.0 6" << endl << endl;

    out << "l1 polygon light1\n0 0 12\n" \
        "1.85 " << hi-eps << " 1 2.15 " << hi-eps << " 1 2.15 " << hi-eps << " 3 1.85 " << hi-eps << " 3" << endl << endl;


    // output walls
    for (int i = 0; i < wf.wallsegments.size(); ++i) {
        out << "wallmat polygon wall" << i << endl << 0 << endl << 0 << endl << 12 << " ";
        double x1 = wf.wallsegments[i].getCoords(wf.wallsegments[i].start).first;
        double z1 = wf.wallsegments[i].getCoords(wf.wallsegments[i].start).second;
        double x2 = wf.wallsegments[i].getCoords(wf.wallsegments[i].end).first;
        double z2 = wf.wallsegments[i].getCoords(wf.wallsegments[i].end).second;
        out << x1 << " " << lo << " " << z1 << " " << x1 << " " << hi << " " << z1 << " ";
        out << x2 << " " << hi << " " << z2 << " " << x2 << " " << lo << " " << z2 << endl << endl;
        out << "wallmat polygon wall2_" << i << endl << 0 << endl << 0 << endl << 12 << " ";
        out << x1 << " " << lo << " " << z1 << " " << x2 << " " << lo << " " << z2 << " ";
        out << x2 << " " << hi << " " << z2 << " " << x1 << " " <<hi << " " << z1 << endl << endl;
    }
    double xmin = m.getMesh()->BBox().XMin();
    double zmin = m.getMesh()->BBox().ZMin();
    double xmax = m.getMesh()->BBox().XMax();
    double zmax = m.getMesh()->BBox().ZMax();
    // output floor
    out << "wallmat polygon ceil" << endl << 0 << endl << 0 << endl << 12 << " ";
    out << xmin << " " << hi << " " << zmin << " " << xmin << " " << hi << " " << zmax << " ";
    out << xmax << " " << hi << " " << zmax << " " << xmax << " " << hi << " " << zmin << endl << endl;
    // output ceiling
    out << "wallmat polygon floor" << endl << 0 << endl << 0 << endl << 12 << " ";
    out << xmin << " " << lo << " " << zmin << " " << xmax << " " << lo << " " << zmin << " ";
    out << xmax << " " << lo << " " << zmax << " " << xmin << " " << lo << " " << zmax << endl << endl;
}
