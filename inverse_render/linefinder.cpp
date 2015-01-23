#include "linefinder.h"
#include <limits>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "RNBasics/RNBasics.h"
#include "R3Shapes/R3Shapes.h"

#include "colorhelper.h"

using namespace cv;
using namespace Eigen;
using namespace std;

const double EPSILON = 1e-5;

cv::Vec3f getPixel(const cv::Mat& img, double px, double py) {
    int x = (int)px;
    int y = (int)py;
    int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REPLICATE);
    int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REPLICATE);
    int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REPLICATE);
    int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REPLICATE);
    float a = px - (float)x;
    float c = py - (float)y;
    float b = ((img.at<cv::Vec3f>(y0, x0)[0] * (1.f - a) + img.at<cv::Vec3f>(y0, x1)[0] * a) * (1.f - c)
            + (img.at<cv::Vec3f>(y1, x0)[0] * (1.f - a) + img.at<cv::Vec3f>(y1, x1)[0] * a) * c);
    float g = ((img.at<cv::Vec3f>(y0, x0)[1] * (1.f - a) + img.at<cv::Vec3f>(y0, x1)[1] * a) * (1.f - c)
            + (img.at<cv::Vec3f>(y1, x0)[1] * (1.f - a) + img.at<cv::Vec3f>(y1, x1)[1] * a) * c);
    float r = ((img.at<cv::Vec3f>(y0, x0)[2] * (1.f - a) + img.at<cv::Vec3f>(y0, x1)[2] * a) * (1.f - c)
            + (img.at<cv::Vec3f>(y1, x0)[2] * (1.f - a) + img.at<cv::Vec3f>(y1, x1)[2] * a) * c);
    return cv::Vec3f(b, g, r);
}

void orientedEdgeFilterVP(char* image, int w, int h, Vector3d vp, vector<Vector4d>& lines, int windowy = 21, int windowx=5) {
    double** window[2];
    for (int k = 0; k < 2; ++k) {
        window[k] = new double*[windowx];
        for (int i = 0; i < windowx; ++i) window[k][i] = new double[windowy];
    }
    Mat img(h, w, CV_32FC3, image);
    flip(img, img, 0);
    float* edges = new float[w*h];

    cvtColor(img, img, CV_BGR2HSV);
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            float maxresponse = 0;
            // Determine window orientation
            Vector2d lk;
            Vector2d p(j,i);
            if (vp[2] == 0) {
                lk = vp.head(2);
            } else {
                lk = vp.head(2) - p;
            }
            if (lk.norm() == 0) {
                maxresponse = numeric_limits<float>::infinity();
            } else {
                lk /= lk.norm();
                Vector2d lpk(lk(1), -lk(0));
                // Get window pixels
                for (int x = 0; x < windowx; ++x) {
                    for (int y = 0; y < windowy; ++y) {
                        double xx = x - windowx/2.;
                        double yy = y - windowy/2.;
                        Vector2d pp = p + xx*lpk + yy*lk;
                        Vec3f pix = getPixel(img, pp[0], pp[1]);
                        for (int k = 0; k < 2; ++k) {
                            window[k][x][y] = pix.val[2*k];
                        }
                    }
                }
                // Get aggregate filter responses
                for (int k = 0; k < 2; ++k) {
                    float response = 0;
                    float presponse = 0;
                    for (int x = 1; x < windowx-1; ++x) {
                        for (int y = 1; y < windowy-1; ++y) {
                            response += abs(window[k][x+1][y] - window[k][x-1][y]);
                            presponse += abs(window[k][x][y+1] - window[k][x][y-1]);
                        }
                    }
                    if (response > 0) {
                        if (presponse == 0) maxresponse = numeric_limits<double>::infinity();
                        else maxresponse = max(maxresponse, response/presponse);
                    }
                }
            }
            int idx = i*w+j;
            edges[idx] = maxresponse;
        }
    }
    ColorHelper::writeExrImage("edges.exr", edges, w, h, 1);
}

void findVanishingPoints(const CameraParams& cam, R4Matrix normalization, vector<Eigen::Vector3d>& vps) {
    R3Vector a = normalization.Inverse()*cam.towards;
    R3Vector b = normalization.Inverse()*cam.up;
    R3Vector c = normalization.Inverse()*cam.right;
    R3Point p = normalization.Inverse()*cam.pos;
    R4Matrix t(
        c[0], b[0], -a[0], p[0],
        c[1], b[1], -a[1], p[1],
        c[2], b[2], -a[2], p[2],
        0, 0, 0, 1
    );
    t = t.Inverse();
    for (int i = 0; i < 3; ++i) {
        R3Vector a =  R3zero_vector;
        a[i] = 1;
        a = t*a;
        if (abs(a[2]) < EPSILON) {
            Vector3d vp(a[0],a[1],0);
            vps.push_back(vp);
        } else {
            Vector3d vp(a[0]/a[2]*cam.focal_length + cam.width/2,
                        a[1]/a[2]*cam.focal_length + cam.height/2,
                        1);
            vps.push_back(vp);
        }
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
    vector<Vector3d> vps;
    findVanishingPoints(cam, norm, vps);
    orientedEdgeFilterVP(image, cam.width, cam.height, vps[1], imglines);
    return;

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

void findWallLines(ColorHelper& ch, WallFinder& wf, vector<WallLine>& lines, double resolution, bool getvotes) {
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
        if (getvotes) {
            for (int j = 0; j < votes[i].size(); ++j) {
                WallLine wl(i, votes[i][j][0]);
                wl.starty = votes[i][j][1];
                wl.endy = votes[i][j][2];
                lines.push_back(wl);
            }
        } else {
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
}
