#include "wall_finder.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <limits>
#include <fstream>

#include "shortestpath.h"

using namespace pcl;
using namespace std;
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename PointT>
class DotComparison : public ComparisonBase<PointT> {
    using ComparisonBase<PointT>::field_name_;
    using ComparisonBase<PointT>::op_;
    using ComparisonBase<PointT>::capable_;

    public:
    typedef boost::shared_ptr< DotComparison<PointT> > Ptr;
    typedef boost::shared_ptr< const DotComparison<PointT> > ConstPtr;
    DotComparison(Eigen::Vector3f vec, ComparisonOps::CompareOp op, double compare_val)
        : ComparisonBase<PointT>()
          , compare_val_(compare_val), dot(vec)
    {
        capable_ = true;
    }
    DotComparison(const DotComparison& src)
        : ComparisonBase<PointT>()
          , compare_val_(src.compare_val_), dot(src.dot)
    {
        capable_ = true;
    }
    /** \brief Copy operator.
     *  \param[in] src the field comparison object to copy into this
     */
    inline DotComparison& operator = (const DotComparison &src)
    {
        compare_val_ = src.compare_val_;
        dot = src.dot;
        return (*this);
    }

    /** \brief Destructor. */
    virtual ~DotComparison () {;}

    /** \brief Determine the result of this comparison.
     * \param point the point to evaluate
     * \return the result of this comparison.
     */
    virtual bool evaluate (const PointT &point) const {
        double d = point.normal_x*dot(0) + point.normal_y*dot(1) + point.normal_z*dot(2);
        switch (this->op_)
        {
            case pcl::ComparisonOps::GT :
                return (d > compare_val_);
            case pcl::ComparisonOps::GE :
                return (d >= compare_val_);
            case pcl::ComparisonOps::LT :
                return (d < compare_val_);
            case pcl::ComparisonOps::LE :
                return (d <= compare_val_);
            case pcl::ComparisonOps::EQ :
                return (d == compare_val_);
            default:
                PCL_WARN ("[pcl::DotComparison::evaluate] unrecognized op_!\n");
                return (false);
        }
    }
    protected:
        double compare_val_;
        Eigen::Vector3f dot;
    private:
        DotComparison() {;}
};
double WallFinder::findExtremalHistogram(
        PointCloud<PointNormal>::ConstPtr cloud,
        Eigen::Vector3f dir,
        double resolution, double threshold)
{
    double vmin = std::numeric_limits<double>::infinity();
    double vmax = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < cloud->points.size(); ++i) {
        double val = cloud->points[i].x*dir(0) + cloud->points[i].y*dir(1) + cloud->points[i].z*dir(2);
        vmin = min(vmin, val);
        vmax = max(vmax, val);
    }
    int sz = 1+(vmax-vmin)/resolution;
    int* hist = new int[sz];
    memset(hist, 0, sz*sizeof(int));
    for (int i = 0; i < cloud->points.size(); ++i) {
        double val = cloud->points[i].x*dir(0) + cloud->points[i].y*dir(1) + cloud->points[i].z*dir(2);
        hist[(int)((val-vmin)/resolution)]++;
    }

    int besti = -1;
    for (int i = 0; i < sz; ++i) {
        if (hist[i] > threshold) {
            besti = i;
            break;
        }
    }
    double avg = 0;
    int n = 0;
    for (int i = 0; i < cloud->points.size(); ++i) {
        double val = cloud->points[i].x*dir(0) + cloud->points[i].y*dir(1) + cloud->points[i].z*dir(2);
        int idx = (val-vmin)/resolution;
        if (idx == besti || idx == besti+1) {
            avg += val;
            n++;
        }
    }
    return avg/n;
}

double WallFinder::findExtremalNormal(
        PointCloud<PointNormal>::ConstPtr cloud,
        Eigen::Vector3f dir,
        double anglethreshold,
        PointCloud<PointNormal>::Ptr outcloud)
{
    ConditionOr<PointNormal>::Ptr cond(new ConditionOr<PointNormal>());
    cond->addComparison(DotComparison<PointNormal>::ConstPtr(new DotComparison<PointNormal>(dir, ComparisonOps::GT, 1. - anglethreshold)));
    ConditionalRemoval<PointNormal> filter(cond);
    filter.setInputCloud(cloud);
    filter.filter(*outcloud);

    // Detect all planes
    ModelCoefficients coefficients;
    PointIndices::Ptr inliers (new PointIndices());
    SACSegmentation<PointNormal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(resolution);
    seg.setAxis(dir);
    seg.setEpsAngle(anglethreshold);
    ExtractIndices<PointNormal> extract;
    int nr_points = outcloud->points.size();
    PointCloud<PointNormal>::Ptr segmented_cloud(new PointCloud<PointNormal>(*outcloud));
    PointCloud<PointNormal>::Ptr swap_cloud(new PointCloud<PointNormal>());
    double best_dist = -numeric_limits<double>::max();
    while (segmented_cloud->points.size() > 0.1*nr_points) {
        seg.setInputCloud (segmented_cloud);
        seg.segment (*inliers, coefficients);
        if (inliers->indices.size () == 0) {
            break;
        }
        Eigen::Vector3f candidate(coefficients.values[0],
                                  coefficients.values[1],
                                  coefficients.values[2]);
        if (candidate.dot(dir) < 0) {
            for (int i = 0; i < 4; ++i) {
                coefficients.values[i] = -coefficients.values[i];
            }
        }
        if (best_dist < coefficients.values[3]) {
            best_dist = coefficients.values[3];
        }
        extract.setInputCloud(segmented_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*swap_cloud);
        segmented_cloud.swap(swap_cloud);
    }
    return -best_dist;
}

double WallFinder::findFloorAndCeiling(
        vector<int>& labels,
        double anglethreshold)
{
    PointCloud<PointNormal>::ConstPtr cloud = of->getCloud();
    floorplane = findExtremalHistogram(cloud, Eigen::Vector3f(0.,1.,0.), resolution, 10000);
    ceilplane = -findExtremalHistogram(cloud, Eigen::Vector3f(0.,-1.,0.), resolution, 10000);
    double t = cos(anglethreshold);
    for (int i = 0; i < cloud->size(); ++i) {
        if (abs((*cloud)[i].y - floorplane) < resolution &&
                (*cloud)[i].normal_y < -t) {
            labels[i] = LABEL_FLOOR;
        } else if (abs((*cloud)[i].y - ceilplane) < resolution &&
                (*cloud)[i].normal_y > t) {
            labels[i] = LABEL_CEILING;
        }
    }
    return floorplane;
}

class Grid {
    public:
        Grid(int w, int h, double r) : width(w), height(h), resolution(r) {
            grid = new GridCell*[width];
            for (int i = 0; i < width; ++i) grid[i] = new GridCell[height];
        }
        void insert(const PointNormal& p, int idx) {
            int r = p.x/resolution;
            int c = p.z/resolution;
            if (r < 0 || c < 0 || r >= width || c >= height) {
                return;
            }
            grid[r][c].add(p, idx);
        }
        vector<int>& getPointIndices(int i, int j) {
            return grid[i][j].pointindices;
        }
        vector<int>& getPointIndices(pair<int,int> p) {
            return grid[p.first][p.second].pointindices;
        }
        int getCount(int i, int j) {
            if (i < 0 || j < 0 || i >= width || j >= height) return 0;
            return grid[i][j].count;
        }
    private:
        class GridCell {
            public:
                GridCell() : count(0), hi(0), lo(9999) {;}
                void add(PointNormal p, int i) {
                    ++count;
                    if (p.y > hi) hi = p.y;
                    if (p.y < lo) lo = p.y;
                    pointindices.push_back(i);
                }
                int count;
                double hi;
                double lo;
                vector<int> pointindices;
        };
        GridCell** grid;
        int width;
        int height;
        double resolution;
};
PointCloud<PointXYZ>::Ptr WallFinder::getHistogram()
{
    // Create grid
    float maxx = -numeric_limits<float>::max();
    float maxz = -numeric_limits<float>::max();
    PointCloud<PointNormal>::const_iterator it;
    for (it = of->getCloud()->begin(); it != of->getCloud()->end(); ++it) {
        if (it->x > maxx) maxx = it->x;
        if (it->z > maxz) maxz = it->z;
    }
    int width = maxx/resolution + 1;
    int height = maxz/resolution + 1;
    Grid grid(width, height, resolution);
    for (int i = 0; i < of->getCloud()->size(); ++i) {
        grid.insert(of->getCloud()->at(i), i);
    }

    pcl::PointCloud<PointXYZ>::Ptr ret(new PointCloud<PointXYZ>());
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            ret->push_back(PointXYZ(i*resolution, sqrt(grid.getCount(i,j)/100.),j*resolution));
        }
    }
    return ret;
}

void WallFinder::findWalls(
        vector<int>& labels,
        int wallthreshold,
        double minlength,
        double anglethreshold,
        bool returnallcandidates)
{
    // Create grid
    float maxx = -numeric_limits<float>::max();
    float maxz = -numeric_limits<float>::max();
    PointCloud<PointNormal>::const_iterator it;
    for (it = of->getCloud()->begin(); it != of->getCloud()->end(); ++it) {
        maxx = max(it->x, maxx);
        maxz = max(it->z, maxz);
    }
    int width = maxx/resolution + 1;
    int height = maxz/resolution + 1;
    Grid grid(width, height, resolution);
    for (int i = 0; i < of->getCloud()->size(); ++i) {
        grid.insert(of->getCloud()->at(i), i);
    }

    // Scan grid and extract line segments
    int overlapthreshold = 10;
    int steepness = 10;
    int skipsallowed = 3;
    vector<GridSegment> segments;
    // Check vertical walls
    int numsegs = 0;
    int skipped = 0;
    for (int i = 0; i < width; ++i) {
        numsegs = 0;
        skipped = 0;
        for (int j = 0; j < height; ++j) {
            if (grid.getCount(i,j) > wallthreshold &&
                grid.getCount(i,j) - grid.getCount(i-1,j) > steepness &&
                grid.getCount(i,j) - grid.getCount(i+1,j) > steepness)
            {
                numsegs += skipped + 1;
                skipped = 0;
            } else {
                if (numsegs && skipped < skipsallowed) {
                    skipped++;
                }
                else {
                    if (numsegs*resolution > minlength) {
                        segments.push_back(GridSegment(0,j-numsegs,j,i));
                    }
                    numsegs = 0;
                    skipped = 0;
                }
            }
        }
        if (numsegs*resolution > minlength) {
            segments.push_back(GridSegment(0,height-numsegs,height,i));
        }
    }
    // Check horizontal walls
    for (int i = 0; i < height; ++i) {
        numsegs = 0;
        skipped = 0;
        for (int j = 0; j < width; ++j) {
            if (grid.getCount(j,i) > wallthreshold &&
                grid.getCount(j,i-1) < grid.getCount(j,i) &&
                grid.getCount(j,i+1) < grid.getCount(j,i))
            {
                numsegs += skipped + 1;
                skipped = 0;
            } else {
                if (numsegs && skipped < skipsallowed) {
                    skipped++;
                }
                else {
                    if (numsegs*resolution > minlength) {
                        segments.push_back(GridSegment(1,j-numsegs,j,i));
                    }
                    numsegs = 0;
                    skipped = 0;
                }
            }
        }
        if (numsegs*resolution > minlength) {
            segments.push_back(GridSegment(1,width-numsegs,width,i));
        }
    }
    // Filter out segments with no empty space
    vector<GridSegment> candidatewalls;
    vector<GridSegment> walls;
    for (int i = 0; i < segments.size(); ++i) {
        int netcount = 0;
        for (int j = segments[i].start; j < segments[i].end; ++j) {
            vector<int>& indices = grid.getPointIndices(segments[i].getCoords(j));
            for (int k = 0; k < indices.size(); ++k) {
                if (segments[i].direction) {
                    netcount += -sgn<double>(of->getCloud()->at(indices[k]).normal_z);
                } else {
                    netcount += -sgn<double>(of->getCloud()->at(indices[k]).normal_x);
                }
            }
        }
        segments[i].norm = sgn<int>(netcount);
        candidatewalls.push_back(segments[i]);
    }

    // Merge collinear segments
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < candidatewalls.size(); ++i) {
            for (int j = 0; j < i; ++j) {
                if (candidatewalls[i].direction == candidatewalls[j].direction &&
                    abs(candidatewalls[i].coord - candidatewalls[j].coord) < 3)
                {
                    bool swapped = false;
                    if (candidatewalls[i].start < candidatewalls[j].start) {
                        swapped = true;
                        swap(candidatewalls[i],candidatewalls[j]);
                    }
                    if (candidatewalls[i].start - candidatewalls[j].end > skipsallowed*2) {
                        continue;
                    }
                    changed = true;
                    if (candidatewalls[i].end > candidatewalls[j].end) {
                        candidatewalls[j].end = candidatewalls[i].end;
                    }
                    int il = candidatewalls[i].end - candidatewalls[i].start;
                    int jl = candidatewalls[j].end - candidatewalls[j].start;
                    candidatewalls[j].coord = (candidatewalls[i].coord*il + candidatewalls[j].coord*jl)/(il+jl);
                    candidatewalls.erase(candidatewalls.begin()+i);
                    --i;
                    break;
                }
            }
        }
    }
    int maxidx = -1;
    double maxlength = 0;
    for (int i = 0; i < candidatewalls.size(); ++i) {
        if (maxlength < candidatewalls[i].end - candidatewalls[i].start) {
            maxlength = candidatewalls[i].end - candidatewalls[i].start;
            maxidx = i;
        }
    }
    if (maxidx == -1) {
        cerr << "Error! No walls found!" << endl;
        return;
    }

    // Create edge cost matrix based on compatibility
    // Two segments are compatible if:
    //     Perpendicular: endpoints are close together and normals are compatible
    //     Parallel: Normals are compatible and coords are similar and close together
    double** edges = new double*[candidatewalls.size()*2];
    for (int i = 0; i < candidatewalls.size(); ++i) {
        edges[2*i] = new double[candidatewalls.size()*2];
        edges[2*i+1] = new double[candidatewalls.size()*2];
        for (int j = 0; j < 2*candidatewalls.size(); ++j) {
            edges[2*i][j] = numeric_limits<double>::quiet_NaN();
            edges[2*i+1][j] = numeric_limits<double>::quiet_NaN();
        }
        edges[2*i+1][2*i] = 0;
        for (int j = 0; j < i; ++j) {
            edges[2*i][2*j] = numeric_limits<double>::max();
            edges[2*i+1][2*j] = numeric_limits<double>::max();
            edges[2*i+1][2*j+1] = numeric_limits<double>::max();
            edges[2*i][2*j+1] = numeric_limits<double>::max();
            if (candidatewalls[i].direction == candidatewalls[j].direction) {
                if (abs(candidatewalls[i].coord - candidatewalls[j].coord) < 3 &&
                    candidatewalls[i].norm == candidatewalls[j].norm)
                {
                    edges[2*i][2*j] = abs(candidatewalls[i].start - candidatewalls[j].start);
                    edges[2*i+1][2*j] = abs(candidatewalls[i].end - candidatewalls[j].start);
                    edges[2*i+1][2*j+1] = abs(candidatewalls[i].end - candidatewalls[j].end);
                    edges[2*i][2*j+1] = abs(candidatewalls[i].start - candidatewalls[j].end);
                }
            } else {
                // HACK: Uses fact that vertical edges are always before horizontal ones
                GridSegment& a = candidatewalls[i];
                GridSegment& b = candidatewalls[j];
                int n = a.norm*b.norm;
                if (b.coord >= a.end - overlapthreshold && a.coord >= b.end - overlapthreshold && n >= 0) {
                    edges[2*i+1][2*j+1] = abs(b.coord - a.end + a.coord - b.end);
                }
                if (b.coord >= a.end - overlapthreshold && a.coord <= b.start + overlapthreshold && n <= 0) {
                    edges[2*i+1][2*j] = abs(b.coord - a.end + b.start - a.coord);
                }
                if (b.coord <= a.start + overlapthreshold && a.coord >= b.end - overlapthreshold && n <= 0) {
                    edges[2*i][2*j+1] = abs(a.start - b.coord + a.coord - b.end);
                }
                if (b.coord <= a.start + overlapthreshold && a.coord <= b.start + overlapthreshold && n >= 0) {
                    edges[2*i][2*j] = abs(a.start - b.coord + b.start - a.coord);
                }
            }
        }
    }
    for (int i = 0; i < 2*candidatewalls.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            if (isnan(edges[i][j])) edges[i][j] = edges[j][i];
            else edges[j][i] = edges[i][j];
        }
    }
    // Force graph to be directed via bfs
    {
        vector<bool> visited(candidatewalls.size()*2, false);
        deque<int> q;
        deque<int> d;
        visited[2*maxidx] = true;
        visited[2*maxidx+1] = true;
        q.push_back(2*maxidx+1);
        d.push_back(0);
        for (int i = 0; i < candidatewalls.size()*2; ++i) {
            edges[2*maxidx][i] = numeric_limits<double>::max();
        }
        edges[2*maxidx][2*maxidx+1] = 0;
        edges[2*maxidx+1][2*maxidx] = numeric_limits<double>::max();
        while (!q.empty()) {
            int n = q.front(); q.pop_front();
            int dist = d.front(); d.pop_front();
            int nn = (n&1)?n-1:n+1;
            if (dist&1) {
                if (!visited[nn]) {
                    q.push_back(nn);
                    d.push_back(dist+1);
                    visited[nn] = true;
                    for (int i = 0; i < candidatewalls.size()*2; ++i) {
                        if (i == nn || i == n) continue;
                        edges[n][i] = numeric_limits<double>::max();
                    }
                    edges[nn][n] = numeric_limits<double>::max();
                }
            } else {
                for (int i = 0; i < candidatewalls.size()*2; ++i) {
                    if (n == i || nn == i) continue;
                    if (edges[n][i] < numeric_limits<double>::max()) {
                        if (!visited[i]) {
                            q.push_back(i);
                            d.push_back(dist+1);
                            visited[i] = true;
                        }
                        edges[i][n] = numeric_limits<double>::max();
                    }
                }
            }
        }
    }
    if (returnallcandidates) {
        for (int i = 0; i < candidatewalls.size(); ++i) {
            wallsegments.push_back(Segment(candidatewalls[i], resolution));
        }
        return;
    }
    // Starting with largest planar section, wind around compatible sections
    vector<int> wall;
    double c = shortestPath(2*maxidx, 2*maxidx, edges, 2*candidatewalls.size(), wall);
    if (c == numeric_limits<double>::max()) {
        cerr << "Error determining floor plan!" << endl;
    }
    for (int i = 0; i < wall.size(); i+=2) {
        wallsegments.push_back(Segment(candidatewalls[wall[i]/2], resolution));
    }
    // Add labels to all compatible pixels regardless of bin and compute more
    // accurate coords
    int i;
    vector<double> segmentcoords(wallsegments.size(), 0);
    vector<int> segmentcounts(wallsegments.size(), 0);
    for (it = of->getCloud()->begin(), i=0; it != of->getCloud()->end(); ++it, ++i) {
        for (int j = 0; j < wallsegments.size(); ++j) {
            bool horiz = wallsegments[j].direction;
            // Check if on same plane
            double coord = horiz?it->z:it->x;
            if (abs(wallsegments[j].coord - coord) > 2*resolution) {
                continue;
            }
            // Check if within bounds
            coord = horiz?it->x:it->z;
            if (coord > 2*resolution + wallsegments[j].end || coord < wallsegments[j].start - 2*resolution) {
                continue;
            }
            // Check if compatible normal
            Eigen::Vector3f normal(it->normal_x,
                                   it->normal_y,
                                   it->normal_z);
            Eigen::Vector3f axis = (horiz?Eigen::Vector3f::UnitZ():Eigen::Vector3f::UnitX());
            axis *= -wallsegments[j].norm;
            if (normal.dot(axis) > cos(anglethreshold)) {
                labels[i] = LABEL_WALL;
                segmentcoords[j] += horiz?it->z:it->x;
                segmentcounts[j]++;
                break;
            }
        }
    }
    for (int i = 0; i < wallsegments.size(); ++i) {
        wallsegments[i].coord = segmentcoords[i]/segmentcounts[i];
    }
    // Make edges meet
    for (int i = 0; i < wallsegments.size(); ++i) {
        int j = (i+1)%wallsegments.size();
        if (wallsegments[i].direction != wallsegments[j].direction) {
            if (abs(wallsegments[i].start - wallsegments[j].coord) <
                abs(wallsegments[i].end - wallsegments[j].coord))
            {
                wallsegments[i].start = wallsegments[j].coord;
            } else {
                wallsegments[i].end = wallsegments[j].coord;
            }
            if (abs(wallsegments[j].start - wallsegments[i].coord) <
                abs(wallsegments[j].end - wallsegments[i].coord))
            {
                wallsegments[j].start = wallsegments[i].coord;
            } else {
                wallsegments[j].end = wallsegments[i].coord;
            }
        } else {
            // Merge parallel segments
            if (wallsegments[i].start > wallsegments[j].start) {
                wallsegments[i].start = wallsegments[j].start;
            }
            if (wallsegments[i].end < wallsegments[j].end) {
                wallsegments[i].end = wallsegments[j].end;
            }
            int il = wallsegments[i].end - wallsegments[i].start;
            int jl = wallsegments[j].end - wallsegments[j].start;
            wallsegments[i].coord = (wallsegments[i].coord*il + wallsegments[j].coord*jl)/(il+jl);
            wallsegments.erase(wallsegments.begin()+j);
            --i;
        }
    }
}

void WallFinder::loadWalls(string filename, vector<int>& labels) {
    ifstream in(filename.c_str(), ifstream::binary);
    uint32_t sz;
    in.read((char*) &sz, 4);
    labels.resize(sz);
    in.read((char*) &sz, 4);
    wallsegments.resize(sz);
    in.read((char*) &resolution, sizeof(double));
    for (int i = 0; i < labels.size(); ++i) {
        in.read((char*) &(labels[i]), 4);
    }
    for (int i = 0; i < sz; ++i) {
        in.read((char*) &(wallsegments[i].direction), 4);
        in.read((char*) &(wallsegments[i].start), sizeof(double));
        in.read((char*) &(wallsegments[i].end), sizeof(double));
        in.read((char*) &(wallsegments[i].norm), 4);
        in.read((char*) &(wallsegments[i].coord), sizeof(double));
    }
    in.read((char*) &floorplane, sizeof(double));
    in.read((char*) &ceilplane, sizeof(double));
}
void WallFinder::saveWalls(string filename, vector<int>& labels) {
    ofstream out(filename.c_str(), ofstream::binary);
    uint32_t sz = labels.size();
    out.write((char*) &sz, 4);
    sz = wallsegments.size();
    out.write((char*) &sz, 4);
    out.write((char*) &resolution, sizeof(double));
    for (int i = 0; i < labels.size(); ++i) {
        out.write((char*) &(labels[i]), 4);
    }
    for (int i = 0; i < wallsegments.size(); ++i) {
        out.write((char*) &(wallsegments[i].direction), 4);
        out.write((char*) &(wallsegments[i].start), sizeof(double));
        out.write((char*) &(wallsegments[i].end), sizeof(double));
        out.write((char*) &(wallsegments[i].norm), 4);
        out.write((char*) &(wallsegments[i].coord), sizeof(double));
    }
    out.write((char*) &floorplane, sizeof(double));
    out.write((char*) &ceilplane, sizeof(double));
}

Eigen::Vector3f WallFinder::getWallEndpoint(int i, bool lo, double height) const {
    if (i >= wallsegments.size()) i %= wallsegments.size();
    Eigen::Vector4f p;
    pair<int,int> x = wallsegments[i].getCoords(lo?wallsegments[i].start:wallsegments[i].end);
    p[0] = x.first;
    p[1] = height*ceilplane + (1-height)*floorplane;
    p[2] = x.second;
    p[3] = 1;
    p = of->getNormalizationTransform().inverse()*p;
    return Eigen::Vector3f(p[0]/p[3], p[1]/p[3], p[2]/p[3]);
}
