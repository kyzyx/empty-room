#include "datamanager/imageio.h"
#include "R3Shapes/R3Shapes.h"
#include <iostream>
#include <string>
#include <map>
#include <vector>

using namespace std;

int main(int argc, char** argv) {
    string line;
    int w, h;
    double vfov;
    int n;
    vector<string> filenames;
    vector<R3Point> p;
    vector<R3Vector> u;
    vector<R3Vector> t;
    // Read in list of filenames
    while (getline(cin, line)) {
        filenames.push_back(line);
        map<string, float> header;
        bool success = ImageIO::readImageHeader(line, w, h, header);

        double m[16];
        m[0] = header["R0"];
        m[1] = header["R1"];
        m[2] = header["R2"];
        m[3] = header["T0"];
        m[4] = header["R3"];
        m[5] = header["R4"];
        m[6] = header["R5"];
        m[7] = header["T1"];
        m[8] = header["R6"];
        m[9] = header["R7"];
        m[10] = header["R8"];
        m[11] = header["T2"];
        m[12] = 0;
        m[13] = 0;
        m[14] = 0;
        m[15] = 1;
        R4Matrix rt(m);
        R4Matrix rtinv = rt.Inverse();
        R3Point pos = rtinv*R3Point(0,0,0);
        p.push_back(pos);
        R3Vector up = rtinv*R3Vector(0,-1,0);
        u.push_back(up);
        R3Vector towards = rtinv*R3Vector(0,0,1);
        t.push_back(towards);

        double foc = header["fy"];
        vfov = atan(0.5/foc)*360/M_PI;
        n++;
    }
    cout << n << " " << w << " " << h << " " << vfov << endl;
    for (int i = 0; i < n; i++) {
        cout << filenames[i] << " ";
        cout << p[i][0] << " " << p[i][1] << " " << p[i][2] << " ";
        cout << u[i][0] << " " << u[i][1] << " " << u[i][2] << " ";
        cout << t[i][0] << " " << t[i][1] << " " << t[i][2] << " ";
        cout << endl;
    }
}
