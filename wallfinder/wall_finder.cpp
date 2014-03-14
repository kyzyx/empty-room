#include "wall_finder.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <limits>

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

double WallFinder::findExtremal(
        PointCloud<PointNormal>::ConstPtr cloud,
        Eigen::Vector3f dir,
        double anglethreshold,
        double resolution,
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
        OrientationFinder& of,
        vector<int>& labels,
        double resolution,
        double anglethreshold)
{
    double floor, ceiling;

    PointCloud<PointNormal>::Ptr floorcandidates(new PointCloud<PointNormal>());
    PointCloud<PointNormal>::ConstPtr cloud = of.getCloud();
    floor = findExtremal(cloud, Eigen::Vector3f(0.,1.,0.), anglethreshold, resolution, floorcandidates);
    PointCloud<PointNormal>::Ptr ceilcandidates(new PointCloud<PointNormal>());
    ceiling = -findExtremal(cloud, Eigen::Vector3f(0.,-1.,0.), anglethreshold, resolution, ceilcandidates);
    double t = cos(anglethreshold);
    for (int i = 0; i < cloud->size(); ++i) {
        if (abs((*cloud)[i].y - floor) < resolution &&
                (*cloud)[i].normal_y > t) {
            labels[i] = LABEL_FLOOR;
        } else if (abs((*cloud)[i].y - ceiling) < resolution &&
                (*cloud)[i].normal_y < -t) {
            labels[i] = LABEL_CEILING;
        }
    }
    return floor;
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
pcl::PointCloud<PointXYZ>::Ptr getHistogram(
        OrientationFinder& of,
        double resolution)
{
    // Create grid
    float maxx = -numeric_limits<float>::max();
    float maxz = -numeric_limits<float>::max();
    PointCloud<PointNormal>::const_iterator it;
    for (it = of.getCloud()->begin(); it != of.getCloud()->end(); ++it) {
        if (it->x > maxx) maxx = it->x;
        if (it->z > maxz) maxz = it->z;
    }
    int width = maxx/resolution + 1;
    int height = maxz/resolution + 1;
    Grid grid(width, height, resolution);
    for (int i = 0; i < of.getCloud()->size(); ++i) {
        grid.insert(of.getCloud()->at(i), i);
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
        OrientationFinder& of,
        vector<int>& labels,
        double minlength,
        double resolution,
        double anglethreshold)
{
    // Create grid
    float maxx = -numeric_limits<float>::max();
    float maxz = -numeric_limits<float>::max();
    PointCloud<PointNormal>::const_iterator it;
    for (it = of.getCloud()->begin(); it != of.getCloud()->end(); ++it) {
        if (it->x > maxx) maxx = it->x;
        if (it->z > maxz) maxz = it->z;
    }
    int width = maxx/resolution + 1;
    int height = maxz/resolution + 1;
    Grid grid(width, height, resolution);
    for (int i = 0; i < of.getCloud()->size(); ++i) {
        grid.insert(of.getCloud()->at(i), i);
    }

    // Scan grid and extract line segments
    int overlapthreshold = 6;
    int wallthreshold = 200;
    int steepness = 10;
    int skipsallowed = 2;
    vector<Segment> segments;
    // Check vertical walls
    int numsegs = 0;
    int skipped = 0;
    for (int i = 0; i < width; ++i) {
        numsegs = 0;
        skipped = 0;
        for (int j = 0; j < height; ++j) {
            if (grid.getCount(i,j) > wallthreshold &&
                grid.getCount(i-1,j) < grid.getCount(i,j) &&
                grid.getCount(i+1,j) < grid.getCount(i,j))
            {
                numsegs += skipped + 1;
                skipped = 0;
            } else {
                if (numsegs && skipped < skipsallowed) {
                    skipped++;
                }
                else {
                    if (numsegs*resolution > minlength) {
                        segments.push_back(Segment(0,j-numsegs,j,i));
                    }
                    numsegs = 0;
                    skipped = 0;
                }
            }
        }
        if (numsegs*resolution > minlength) {
            segments.push_back(Segment(0,height-numsegs,height,i));
        }
    }
    // Check horizontal walls
    for (int i = 0; i < height; ++i) {
        numsegs = 0;
        skipped = 0;
        for (int j = 0; j < width; ++j) {
            if (grid.getCount(j,i) > wallthreshold &&
                grid.getCount(j,i) - grid.getCount(j,i-1) > steepness &&
                grid.getCount(j,i) - grid.getCount(j,i+1) > steepness)
            {
                numsegs += skipped + 1;
                skipped = 0;
            } else {
                if (numsegs && skipped < skipsallowed) {
                    skipped++;
                }
                else {
                    if (numsegs*resolution > minlength) {
                        segments.push_back(Segment(1,j-numsegs,j,i));
                    }
                    numsegs = 0;
                    skipped = 0;
                }
            }
        }
        if (numsegs*resolution > minlength) {
            segments.push_back(Segment(1,width-numsegs,width,i));
        }
    }
    // Filter out segments with no empty space
    vector<Segment> candidatewalls;
    vector<Segment> walls;
    int maxidx = -1;
    double maxlength = 0;
    for (int i = 0; i < segments.size(); ++i) {
        int netcount = 0;
        for (int j = segments[i].start; j < segments[i].end; ++j) {
            vector<int>& indices = grid.getPointIndices(segments[i].getCoords(j));
            for (int k = 0; k < indices.size(); ++k) {
                if (segments[i].direction) {
                    netcount += -sgn<double>(of.getCloud()->at(indices[k]).normal_z);
                } else {
                    netcount += -sgn<double>(of.getCloud()->at(indices[k]).normal_x);
                }
            }
        }
        segments[i].norm = sgn<int>(netcount);
        if (maxlength < segments[i].end - segments[i].start) {
            maxlength = segments[i].end - segments[i].start;
            maxidx = i;
        }
        candidatewalls.push_back(segments[i]);
    }
    // Find discontinuities and fill them in
    // Create edge cost matrix based on compatibility
    // Two segments are compatible if:
    //     Perpendicular: endpoints are close together and normals are compatible
    //     Parallel: Normals are compatible and coords are similar and close together
    double** edges = new double*[candidatewalls.size()];
    for (int i = 0; i < candidatewalls.size(); ++i) {
        edges[i] = new double[candidatewalls.size()];
        for (int j = 0; j < i; ++j) {
            if (candidatewalls[i].direction == candidatewalls[j].direction) {
                if (abs(candidatewalls[i].coord - candidatewalls[j].coord) < 3 &&
                    candidatewalls[i].norm == candidatewalls[j].norm)
                {
                    edges[i][j] = min(
                            abs(candidatewalls[i].start - candidatewalls[j].end),
                            abs(candidatewalls[j].start - candidatewalls[i].end)
                    );
                } else {
                    edges[i][j] = numeric_limits<double>::max();
                }
            } else {
                // HACK: Uses fact that vertical edges are always before horizontal ones
                Segment& a = candidatewalls[i];
                Segment& b = candidatewalls[j];
                int n = a.norm*b.norm;
                if (b.coord >= a.end - overlapthreshold && a.coord >= b.end - overlapthreshold && n >= 0) {
                    edges[i][j] = b.coord - a.end + a.coord - b.end;
                } else if (b.coord >= a.end - overlapthreshold && a.coord <= b.start + overlapthreshold && n <= 0) {
                    edges[i][j] = b.coord - a.end + b.start - a.coord;
                } else if (b.coord <= a.start + overlapthreshold && a.coord >= b.end - overlapthreshold && n <= 0) {
                    edges[i][j] = a.start - b.coord + a.coord - b.end;
                } else if (b.coord <= a.start + overlapthreshold && a.coord <= b.start + overlapthreshold && n >= 0) {
                    edges[i][j] = a.start - b.coord + b.start - a.coord;
                } else {
                    edges[i][j] = numeric_limits<double>::max();
                }
                edges[i][j] = abs(edges[i][j]);
            }
        }
    }
    // Starting with largest planar section, wind around compatible sections
    if (maxidx == -1) {
        cerr << "Error! No walls found!" << endl;
        return;
    }
    int curridx = maxidx;
    vector<int> wall;
    vector<bool> inwall(candidatewalls.size(), false);
    do {
        double mindist = numeric_limits<double>::max();
        int besti;
        for (int i = 0; i < candidatewalls.size(); ++i) {
            if (i == curridx || wall.size() && i == wall.back()) continue;
            double d;
            if (curridx < i) {
                d = edges[i][curridx];
            } else if (curridx > i) {
                d = edges[curridx][i];
            }
            if (d < mindist) {
                mindist = d;
                besti = i;
            }
        }
        wall.push_back(curridx);
        wallsegments.push_back(candidatewalls[curridx]);
        inwall[curridx] = true;
        curridx = besti;
        if (inwall[curridx]) {
            break;
        }
    } while(curridx != maxidx);
    if (curridx != maxidx) {
        cerr << "Error determining floor plan!" << endl;
    }
    // TODO: Extend walls to meet to obtain accurate floor plan
    // Add labels to all compatible pixels regardless of bin
    int i;
    for (it = of.getCloud()->begin(), i=0; it != of.getCloud()->end(); ++it, ++i) {
        for (int j = 0; j < wallsegments.size(); ++j) {
            bool horiz = wallsegments[j].direction;
            // Check if on same plane
            double coord = horiz?it->z:it->x;
            if (abs(resolution*wallsegments[j].coord - coord) > 2*resolution) {
                continue;
            }
            // Check if within bounds
            coord = horiz?it->x:it->z;
            if (coord > resolution*(wallsegments[j].end + 2) || coord < resolution*(wallsegments[j].start - 2)) {
                continue;
            }
            // Check if compatible normal
            Eigen::Vector3f normal(it->normal_x,
                                   it->normal_y,
                                   it->normal_z);
            Eigen::Vector3f axis = (horiz?Eigen::Vector3f::UnitZ():Eigen::Vector3f::UnitX());
            axis *= -wallsegments[j].norm;
            if (acos(normal.dot(axis)) < anglethreshold) {
                labels[i] = LABEL_WALL;
                break;
            }
        }
    }
}
