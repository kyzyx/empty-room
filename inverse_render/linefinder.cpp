#include "linefinder.h"
#include "orientededgefilter.h"
#include <limits>
#include "RNBasics/RNBasics.h"
#include "R3Shapes/R3Shapes.h"

using namespace Eigen;
using namespace std;

const double margin = 0.08;
// ---------------------------------------------------------------------------
// Edge- and line-finding functions for single images
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
                //lookup->addVote(image[3*((h-i-1)*w+j)+v], j, i, minw);
                lookup->addVote(image[3*(i*w+j)+v], j, i, minw);
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
               + (x - cam.width/2 - 0.5)*cam.right
               + (y - cam.height/2 - 0.5)*cam.up;
    R3Point p2 = p1+v;
    p1 = normalization*p1;
    p2 = normalization*p2;

    R3Vector n = s.direction?R3posx_vector:R3posz_vector;
    R3Plane plane(n*s.norm, -s.coord*s.norm);
    R3Ray ray(p1, p2);
    R3Point hit;
    RNClassID rncid = R3Intersects(ray, plane, &hit);
    if (rncid && hit.Y() < ceilingplane+margin && hit.Y() > floorplane-margin) {
        double x = s.direction?hit.Z():hit.X();
        return Vector3d(x - s.start, hit.Y(), (hit-cam.pos).Length());
    } else {
        return Vector3d(-numeric_limits<double>::infinity(), 0, numeric_limits<double>::infinity());
    }
}

Eigen::Vector3d projectOntoFloorplan(
        double x, double y,
        const CameraParams& cam,
        WallFinder& wf)
{
    Matrix4f m4dn = wf.getNormalizationTransform();
    R4Matrix norm(
            m4dn(0,0), m4dn(0,1), m4dn(0,2), m4dn(0,3),
            m4dn(1,0), m4dn(1,1), m4dn(1,2), m4dn(1,3),
            m4dn(2,0), m4dn(2,1), m4dn(2,2), m4dn(2,3),
            m4dn(3,0), m4dn(3,1), m4dn(3,2), m4dn(3,3)
            );
    Vector3d best(1,1,numeric_limits<double>::infinity());
    int bestj = -1;
    for (int j = 0; j < wf.wallsegments.size(); j++) {
        Eigen::Vector3d p = projectOntoWall(
                x, y, cam,
                wf.ceilplane, wf.floorplane,
                norm,
                wf.wallsegments[j]);
        if (p[0] < margin || p[0] > wf.wallsegments[j].length() - margin)
            continue;
        if (p[2] < best[2]) {
            best = p;
            bestj = j;
        }
    }
    return Vector3d(bestj, best[0], best[1]);
}

void findWallLinesInImage(
        ImageManager& imgr,
        int idx,
        WallFinder& wf,
        double resolution,
        R4Matrix norm,
        vector<vector<Vector3d> >& votes)
{
    const CameraParams& cam = *(imgr.getCamera(idx));
    const char* image = (const char*) imgr.getImage(idx);
    vector<Vector4d> lines;
    vector<Vector3d> vps;

    findVanishingPoints(cam, norm, vps);
    if (!(imgr.getFlags("edges", idx) & ImageManager::DF_INITIALIZED)) {
        cerr << "Please precompute edge images before linefinding" << endl;
        return;
    }
    if (!(imgr.getFlags("labels", idx) & ImageManager::DF_INITIALIZED)) {
        cerr << "Please precompute label images before linefinding" << endl;
        return;
    }
    VPHough((const float*) imgr.getImage("edges", idx), (const char*) imgr.getImage("labels", idx), cam.width, cam.height, vps, 1, lines);

    for (int k = 0; k < lines.size(); ++k) {
        cout << ">>data:" << idx << " " << lines[k][0] << " " <<  lines[k][1] << " " << lines[k][2] << " " << lines[k][3] << endl;
        Vector3d bestpt[2];
        Vector3d start = projectOntoFloorplan(
                    lines[k][0], lines[k][1],
                    cam, wf);
        Vector3d end = projectOntoFloorplan(
                    lines[k][2], lines[k][3],
                    cam, wf);
        if (start[0] == end[1] && start[0] >= 0) {
            Vector3d seg((start[1]+end[1])/2,
                         start[2],
                         end[2]);
            votes[start[0]].push_back(seg);
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

void findWallLines(ImageManager& imgr, WallFinder& wf, vector<WallLine>& lines, double resolution, bool getvotes) {
    vector<vector<Vector3d> > votes(wf.wallsegments.size());
    Matrix4f m4dn = wf.getNormalizationTransform();
    R4Matrix norm(
        m4dn(0,0), m4dn(0,1), m4dn(0,2), m4dn(0,3),
        m4dn(1,0), m4dn(1,1), m4dn(1,2), m4dn(1,3),
        m4dn(2,0), m4dn(2,1), m4dn(2,2), m4dn(2,3),
        m4dn(3,0), m4dn(3,1), m4dn(3,2), m4dn(3,3)
    );
    for (int i = 0; i < imgr.size(); ++i) {
        findWallLinesInImage(imgr, i, wf, resolution, norm, votes);
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
