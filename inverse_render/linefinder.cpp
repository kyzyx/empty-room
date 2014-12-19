#include "linefinder.h"
#include <limits>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "RNBasics/RNBasics.h"
#include "R3Shapes/R3Shapes.h"

using namespace cv;
using namespace Eigen;
using namespace std;

void findLines(char* image, int w, int h, std::vector<Eigen::Vector4d>& lines) {
    Mat img(h, w, CV_32FC3, image);
    Mat gray, edges;
    cvtColor(img, gray, CV_BGR2GRAY);
    blur(gray, gray, Size(5,5));
    double ratio = 3;
    double lo = 7.4;
    Canny(gray, edges, lo, ratio*lo, 7);
    vector<Vec4i> houghlines;
    HoughLinesP(edges, houghlines, 1, CV_PI/90, 40, 25, 10);
    for(int i = 0; i < houghlines.size(); ++i) {
        Vec4i l = houghlines[i];
        lines.push_back(Vector4d(l[0], l[1], l[2], l[3]));
    }
}

Vector2d projectOntoWall(double x, double y, const CameraParams& cam, double ceilingplane, double floorplane, R4Matrix normalization, Segment& s) {
    R3Point p1 = cam.pos;
    R3Vector v = cam.focal_length*cam.towards
               + (x - cam.width/2 - 0.5)*cam.right
               + (y - cam.height/2 - 0.5)*cam.up;
    R3Point p2 = p1+v;
    p1 = normalization*p1;
    p2 = normalization*p2;

    R3Vector n = s.direction?R3posz_vector:R3posx_vector;
    R3Plane plane(n*s.norm, -s.coord*s.norm);
    R3Point hit;
    if (R3Intersects(R3Ray(p1,p2), plane, &hit) && hit.Y() < ceilingplane && hit.Y() > floorplane) {
        double x = s.direction?hit.X():hit.Z();
        return Vector2d(x - s.start, hit.Y());
    } else {
        return Vector2d(-numeric_limits<double>::infinity(), 0);
    }
}

void findWallLinesInImage(
        const CameraParams& cam,
        char* image,
        WallFinder& wf,
        double resolution,
        R4Matrix norm,
        vector<vector<Vector3d> >& votes)
{
    vector<Vector4d> imglines;
    findLines(image, cam.width, cam.height, imglines);

    for (int j = 0; j < votes.size(); ++j) {
        for (int k = 0; k < imglines.size(); ++k) {
            // Project endpoints onto wall
            Vector2d aa = projectOntoWall(
                    imglines[k][0], imglines[k][1],
                    cam, wf.ceilplane, wf.floorplane,
                    norm, wf.wallsegments[j]
                    );
            double a = aa(0);
            Vector2d bb = projectOntoWall(
                    imglines[k][2], imglines[k][3],
                    cam, wf.ceilplane, wf.floorplane,
                    norm, wf.wallsegments[j]
                    );
            double b = bb(0);
            // Prune lines that don't intersect the wall
            // Also prune lines that correspond to wall edges
            double margin = 0.04;
            if (a < margin || b < margin) continue;
            if (a > wf.wallsegments[j].length()-margin || b > wf.wallsegments[j].length()-margin)
                continue;
            // Prune non-manhattan lines
            if (abs(a-b) > 1.5*resolution) continue;
            votes[j].push_back(Vector3d((a+b)/2,aa[1],bb[1]));
        }
    }
}

bool veccmp(Vector3d a, Vector3d b) {
    return a[0] < b[0];
}

void findPeaks(vector<Vector3d>& votes, vector<Vector3d>& results, double resolution, double threshold) {
    sort(votes.begin(), votes.end(), veccmp);
    int len = 1 + (votes.back()[0] - votes[0][0])/resolution;
    vector<int> histogram(len,0);
    vector<double> avg(len,0);
    vector<double> top(len,-numeric_limits<double>::infinity());
    vector<double> bot(len,numeric_limits<double>::infinity());
    for (int i = 0; i < votes.size(); ++i) {
        int idx = (votes[i][0]-votes[0][0])/resolution;
        histogram[idx]++;
        avg[idx] += votes[i][0];
        if (votes[i][1] > votes[i][2]) {
            top[idx] = max(top[idx], votes[i][1]);
            bot[idx] = min(bot[idx], votes[i][2]);
        } else {
            top[idx] = max(top[idx], votes[i][2]);
            bot[idx] = min(bot[idx], votes[i][1]);
        }
    }

    // Non-maximum suppression
    vector<bool> ismax(len, true);
    for (int i = 0; i < len; ++i) {
        if (i < len-1 && histogram[i] < histogram[i+1]) ismax[i] = false;
        else if (i > 0 && histogram[i] < histogram[i-1]) ismax[i] = false;
    }

    //int mincount = min(1,(int)(threshold*votes.size()));
    int mincount = 3;
    for (int i = 0; i < len; ++i) {
        if (ismax[i] && histogram[i] > mincount) {
            results.push_back(Vector3d(avg[i]/histogram[i], top[i], bot[i]));
        }
    }
}

void findWallLines(ColorHelper& ch, WallFinder& wf, vector<WallLine>& lines, double resolution) {
    vector<vector<Vector3d> > votes(wf.wallsegments.size());
    Matrix4f m4dn = wf.getNormalizationTransform();
    R4Matrix norm(
        m4dn(0,0), m4dn(0,1), m4dn(0,2), m4dn(0,3),
        m4dn(1,0), m4dn(1,1), m4dn(1,2), m4dn(1,3),
        m4dn(2,0), m4dn(2,1), m4dn(2,2), m4dn(2,3),
        m4dn(3,0), m4dn(3,1), m4dn(3,2), m4dn(3,3)
    );
    for (int i = 0; i < ch.size(); ++i) {
        findWallLinesInImage(*(ch.getCamera(i)), ch.getImage(i), wf, resolution, norm, votes);
    }
    for (int i = 0; i < votes.size(); ++i) {
        if (votes[i].empty()) continue;
        vector<Vector3d> results;
        findPeaks(votes[i], results, resolution, 0.25);
        for (int j = 0; j < results.size(); ++j) {
            WallLine wl(i, results[j][0]);
            wl.starty = results[j][1];
            wl.endy = results[j][2];
            lines.push_back(wl);
        }
    }
}
