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
    double best_dist = numeric_limits<double>::min();
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
                coefficients.values[i] = - coefficients.values[i];
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
        double resolution)
{
    double anglethreshold = 0.01;
    double floor, ceiling;

    PointCloud<PointNormal>::Ptr floorcandidates(new PointCloud<PointNormal>());
    PointCloud<PointNormal>::ConstPtr cloud = of.getCloud();
    floor = findExtremal(cloud, Eigen::Vector3f(0.,1.,0.), anglethreshold, resolution, floorcandidates);
    PointCloud<PointNormal>::Ptr ceilcandidates(new PointCloud<PointNormal>());
    ceiling = findExtremal(cloud, Eigen::Vector3f(0.,-1.,0.), anglethreshold, resolution, ceilcandidates);
    for (int i = 0; i < cloud->size(); ++i) {
        if (abs((*cloud)[i].y - floor) < resolution &&
                (*cloud)[i].normal_y > 1. - anglethreshold) {
            labels[i] = LABEL_FLOOR;
        } else if (abs((*cloud)[i].y - floor) < resolution &&
                (*cloud)[i].normal_y < -1. + anglethreshold) {
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
        void insert(const PointNormal& p) {
            int r = p.x/resolution;
            int c = p.z/resolution;
            if (r < 0 || c < 0 || r >= width || c >= width) {
                return;
            }
            grid[r][c].add(p);
        }
        int getCount(int i, int j) {
            if (i < 0 || j < 0 || i >= width || j >= height) return 0;
            return grid[i][j].count;
        }
    private:
        class GridCell {
            public:
                GridCell() : count(0), hi(0), lo(9999) {;}
                void add(PointNormal p) {
                    ++count;
                    if (p.y > hi) hi = p.y;
                    if (p.y < lo) lo = p.y;
                }
                int count;
                double hi;
                double lo;
        };
        GridCell** grid;
        int width;
        int height;
        double resolution;
};

class Segment {
    public:
    Segment(int d, int s, int e, int c) : direction(d), start(s), end(e), coord(c) {;}
    int direction;
    int start;
    int end;
    int coord;
};
void WallFinder::findWalls(
        OrientationFinder& of,
        vector<int>& labels,
        double minlength,
        double resolution)
{
    // Create grid
    float maxx = numeric_limits<float>::min();
    float maxz = numeric_limits<float>::min();
    PointCloud<PointNormal>::const_iterator it;
    for (it = of.getCloud()->begin(); it != of.getCloud()->end(); ++it) {
        maxx = max(it->x, maxx);
        maxz = max(it->z, maxz);
    }
    int width = maxx/resolution + 1;
    int height = maxz/resolution + 1;
    Grid grid(width, height, resolution);
    for (it = of.getCloud()->begin(); it != of.getCloud()->end(); ++it) {
        grid.insert(*it);
    }

    // Scan grid and extract line segments
    int wallthreshold = 10;
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
    // Add all segments bordering empty space
    // Find discontinuities and fill them in
    // Label wall points
}
