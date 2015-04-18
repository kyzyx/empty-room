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

const double margin = 0.08;
const double EPSILON = 1e-5;
// ---------------------------------------------------------------------------
// Edge- and line-finding functions for single images
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

class HoughLookup {
    public:
        HoughLookup(int width, int height, Vector3d vanishingpoint)
            : w(width), h(height), vp(vanishingpoint)
        {
            int dim = min(w,h)/3;
            weights.resize(dim,0);
            total.resize(dim,0);
            distances.resize(dim);
            sorted.resize(dim,false);
        }

        virtual void addVote(double w, int x, int y, double minweight=numeric_limits<double>::infinity()) {
            // FIXME
        }

        void traceLines(int bin, vector<Vector4d>& lines, double minlength = 20, double skip=1.5) {
            sortDistances(bin);
            sortDistances(bin+1);
            vector<pair<double, int> > d(distances[bin].size() + distances[bin+1].size());
            merge(distances[bin].begin(), distances[bin].end(),
                  distances[bin+1].begin(), distances[bin+1].end(),
                  d.begin());
            double start = 0;
            d.push_back(make_pair(numeric_limits<double>::infinity(), -1));
            for (int i = 1; i < d.size(); ++i) {
                if (d[i].first - d[i-1].first > skip) {
                    if (d[i-1].first - d[start].first > minlength) {
                        int x1 = d[start].second%w;
                        int y1 = d[start].second/w;
                        int x2 = d[i-1].second%w;
                        int y2 = d[i-1].second/w;
                        Vector4d line(x1,y1,x2,y2);
                        double err = lineToVp(line);
                        double linelength = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
                        if (err*err < linelength) {
                            lines.push_back(line);
                        }
                    }
                    start = i;
                }
            }
        }

        int size() const { return weights.size()-1; }
        double getWeight(int i) const {
            if (i < 0 || i >= weights.size()) return 0;
            return weights[i]+weights[i+1];
        }
        double getTotal(int i) const { return total[i] + total[i+1]; }
        double getBinAvg(int i) const { return (total[i]+total[i+1])/(weights[i]+weights[i+1]); }

        int getBinCount(int bin) const { return distances[bin].size(); }
        Vector2d getPointInBin(int bin, int idx) {
            sortDistances(bin);
            int i = idx<0?distances[bin].size()+idx:idx;
            int n = distances[bin][i].second;
            return Vector2d(n%w, (int) (n/w));
        }

    protected:
        double lineToVp(Vector4d line) {
            return R3Distance(R3Line(line[0], line[1], 0, line[2], line[3], 0),R3Point(vp[0],vp[1],0));

        }
        void sortDistances(int bin) {
            if (!sorted[bin]) {
                sorted[bin] = true;
                sort(distances[bin].begin(), distances[bin].end());
            }
        }
        Vector3d vp;
        int w, h;
        vector<vector<pair<double,int> > > distances;
        vector<bool> sorted;
        vector<double> weights;
        vector<double> total;
};

class HoughAngleLookup : public HoughLookup {
    public:
        HoughAngleLookup(int width, int height, Vector3d vanishingpoint)
            : HoughLookup(width, height, vanishingpoint)
        {
            int dim = min(w,h)/6;
            weights.resize(dim,0);
            total.resize(dim,0);
            distances.resize(dim);
            calculateMinMax();
        }
        void addVote(double wt, int x, int y, double minweight=numeric_limits<double>::infinity()) {
            if (wt < minweight) return;
            vector<double> angles;
            double a = getAngle(x,y);
            angles.push_back(getAngle(x-0.5,y-0.5));
            angles.push_back(getAngle(x+0.5,y-0.5));
            angles.push_back(getAngle(x+0.5,y+0.5));
            angles.push_back(getAngle(x-0.5,y+0.5));
            int start = getBinForAngle(*(max_element(angles.begin(), angles.end())));
            int end = getBinForAngle(*(min_element(angles.begin(), angles.end())));
            if (wt > minweight*minweight) wt = minweight*minweight;
            for (int i = start; i <= end; ++i) {
                weights[i] += wt;
                total[i] += a*wt;
                if (wt > minweight) {
                    double d = (vp.head(2) - Vector2d(x,y)).norm();
                    distances[i].push_back(make_pair(d,y*w+x));
                }
            }
        }

        double getBinCenter(int i) const {
            static double inc = (maxa-mina)/(weights.size()-1);
            return inc*i+mina;
        }

    private:
        void calculateMinMax() {
            vector<double> angles;
            angles.push_back(getAngle( -0.5,-0.5));
            angles.push_back(getAngle(w+0.5,-0.5));
            angles.push_back(getAngle(w+0.5,h+0.5));
            angles.push_back(getAngle( -0.5,h+0.5));
            maxa = *(max_element(angles.begin(), angles.end()));
            mina = *(min_element(angles.begin(), angles.end()));
        }
        double getAngle(double x, double y) {
            Vector2d d = vp.head(2) - Vector2d(x,y);
            d /= d.norm();
            double a = atan2(d[1], d[0]);
            if (a < 0) a += 2*M_PI;
            return a;
        }
        int getBinForAngle(double a) {
            return (int) ((a-mina)/(maxa-mina)*(weights.size()-1)+0.5);
        }

        double maxa;
        double mina;
};
void VPHough(const float* image, const char* labelimage, int w, int h, vector<Vector3d>& vps, int v, vector<Vector4d>& lines) {
    Vector3d vp = vps[v];
    HoughLookup* lookup;
    if (vp[2] == 0) lookup = new HoughLookup(w,h,vp);
    else            lookup = new HoughAngleLookup(w,h,vp);

    // Vote for angles
    double minw = 5;
    double minlength = 50;
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            if (!labelimage || labelimage[i*w+j] == WallFinder::LABEL_WALL) {
                lookup->addVote(image[3*((h-i-1)*w+j)+v], j, i, minw);
            }
        }
    }

    // Find local maxima
    vector<bool> localmax(lookup->size(),false);
    for (int i = 0; i < lookup->size(); ++i) {
        double currw = lookup->getWeight(i);
        if (currw >= 2*minw*minlength) {
            if (currw > lookup->getWeight(i-1) && currw > lookup->getWeight(i+1)) {
                localmax[i] = true;
            }
        }
    }
    for (int i = 0; i < lookup->size(); ++i) {
        //if (localmax[i]) {
        if (lookup->getWeight(i) > 2*minw*minlength) {
            lookup->traceLines(i, lines, minlength);
        }
    }
}

// Note: returns edge image
float* orientedEdgeFilterVP(char* image, int w, int h, vector<Vector3d>& vps, int windowy = 21, int windowx=5) {
    double** window[2];
    for (int k = 0; k < 2; ++k) {
        window[k] = new double*[windowx];
        for (int i = 0; i < windowx; ++i) window[k][i] = new double[windowy];
    }
    Mat img(h, w, CV_32FC3, image);
    flip(img, img, 0);
    float* edges = new float[3*w*h];

    cvtColor(img, img, CV_BGR2HSV);
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            for (int v = 0; v < vps.size(); ++v) {
                Vector3d vp = vps[v];
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
                int idx = (h-i-1)*w+j;
                edges[3*idx+v] = maxresponse;
            }
        }
    }
    return edges;
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

// ---------------------------------------------------------------------------
// Line consensus functions
Vector3d projectOntoWall(
        double x, double y,
        const CameraParams& cam,
        double ceilingplane, double floorplane,
        R4Matrix normalization,
        Segment& s)
{
    R3Point p1 = cam.pos;
    R3Vector v = cam.focal_length*cam.towards
               - (x - cam.width/2 - 0.5)*cam.right
               - (y - cam.height/2 - 0.5)*cam.up;
    R3Point p2 = p1+v;
    p1 = normalization*p1;
    p2 = normalization*p2;

    R3Vector n = s.direction?R3posz_vector:R3posx_vector;
    R3Plane plane(n*s.norm, -s.coord*s.norm);
    R3Point hit;
    if (R3Intersects(R3Ray(p1,p2), plane, &hit) && hit.Y() < ceilingplane+margin && hit.Y() > floorplane-margin) {
        double x = s.direction?hit.X():hit.Z();
        return Vector3d(x - s.start, hit.Y(), (hit-cam.pos).Length());
    } else {
        return Vector3d(-numeric_limits<double>::infinity(), 0, numeric_limits<double>::infinity());
    }
}

void findWallLinesInImage(
        ColorHelper& ch,
        int idx,
        WallFinder& wf,
        double resolution,
        R4Matrix norm,
        vector<vector<Vector3d> >& votes)
{
    const CameraParams& cam = *(ch.getCamera(idx));
    char* image = ch.getImage(idx);
    vector<Vector4d> verticallines;
    vector<Vector3d> vps;

    findVanishingPoints(cam, norm, vps);
    if (!ch.getEdges(idx)) {
        float* edges = orientedEdgeFilterVP(image, cam.width, cam.height, vps);
        ch.setEdges(idx, edges);
    }
    VPHough(ch.getEdges(idx), ch.getLabelImage(idx), cam.width, cam.height, vps, 1, verticallines);

    for (int k = 0; k < verticallines.size(); ++k) {
        Vector3d bestpt[2];
        int bestwall[2];
        bestwall[0] = -1;
        bestwall[1] = -1;
        double closest[2];
        closest[0] = numeric_limits<double>::infinity();
        closest[1] = numeric_limits<double>::infinity();
        for (int j = 0; j < votes.size(); ++j) {
            for (int i = 0; i < 2; ++i) {
                Vector3d x = projectOntoWall(
                        verticallines[k][2*i], verticallines[k][2*i+1],
                        cam, wf.ceilplane, wf.floorplane,
                        norm, wf.wallsegments[j]
                        );
                // Prune points that don't intersect the wall
                if (x[0] < margin || x[0] > wf.wallsegments[j].length() - margin)
                    continue;
                if (x[2] < closest[i]) {
                    bestpt[i] = x;
                    bestwall[i] = j;
                    closest[i] = x[2];
                }
            }
        }
        if (bestwall[0] == bestwall[1] && bestwall[0] >= 0) {
            Vector3d seg((bestpt[0][0]+bestpt[1][0])/2,
                         bestpt[0][1],
                         bestpt[1][1]);
            votes[bestwall[0]].push_back(seg);
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
    int mincount = 5;
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
        findWallLinesInImage(ch, i, wf, resolution, norm, votes);
        cout << "Done linefinding image " << i << endl;
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
