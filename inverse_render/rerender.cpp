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
    // RADIANCE light parameters are in radiance; for diffuse surfaces the radiance L
    // is related to the radiosity J by
    // J = pi*L
    for (int i = 0; i < ir.lights.size(); ++i) {
        out << "void light l" << i << endl << 0 << endl << 0 << endl << 3 << " ";
        out << ir.lights[i](0)/M_PI << " " << ir.lights[i](1)/M_PI << " " << ir.lights[i](2)/M_PI << endl << endl;
    }

    // Output light triangles;
    for (int i = 0; i < m.getMesh()->NFaces(); ++i) {
        R3MeshFace* f = m.getMesh()->Face(i);
        int lightid = -1;
        // Check if face is a light
        for (int j = 0; j < 3; ++j) {
            int n = m.getMesh()->VertexID(m.getMesh()->VertexOnFace(f, j));
            if (m.labels[n] < 0) {
                lightid = -1;
                break;
            } else {
                if (lightid < 0) lightid = m.labels[n];
                else if (lightid != m.labels[n]) {
                    lightid = -1;
                    break;
                }
            }
        }
        if (lightid == -1) continue;
        out << "l" << lightid << " polygon light" << i << endl << "0 0 9" << endl;
        for (int j = 0; j < 3; ++j) {
            double b[3];
            b[0] = 0; b[1] = 0; b[2] = 0;
            b[j] = 1;
            R3Point v = m.getMesh()->FaceBarycentric(f, b);
            out << v[0] << " " << v[1] << " " << v[2] << " ";
        }
        out << endl << endl;
    }

    double hi = wf.ceilplane;
    double lo = wf.floorplane;

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
    // Output floor and ceiling
    out << "wallmat polygon ceil" << endl << 0 << endl << 0 << endl << 12 << " ";
    out << xmin << " " << hi << " " << zmin << " " << xmin << " " << hi << " " << zmax << " ";
    out << xmax << " " << hi << " " << zmax << " " << xmax << " " << hi << " " << zmin << endl << endl;
    out << "wallmat polygon floor" << endl << 0 << endl << 0 << endl << 12 << " ";
    out << xmin << " " << lo << " " << zmin << " " << xmax << " " << lo << " " << zmin << " ";
    out << xmax << " " << lo << " " << zmax << " " << xmin << " " << lo << " " << zmax << endl << endl;
}
