#include "util.h"
#include <limits>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>

using namespace Eigen;
using namespace pcl;
using namespace std;

static DefaultPointRepresentation<PointXYZ> pr;
bool isValid(PointXYZ p) { return pr.isValid(p); }
double dist2(PointXYZ a, PointXYZ b) {
    if (!pr.isValid(a) || !pr.isValid(b)) return numeric_limits<double>::infinity();
    return (Vector3f(a.x,a.y,a.z)-Vector3f(b.x,b.y,b.z)).squaredNorm();
}

Vector4d transformPlane(Vector4d plane, Matrix4d transform)
{
    Vector4d dir = plane;
    dir(3) = 0;
    Vector4d p(-plane(3)*plane(0), -plane(3)*plane(1), -plane(3)*plane(2), 1);

    dir = transform*dir;
    dir.head(3) = dir.head(3).normalized();
    p = transform*p;

    dir(3) = -dir.dot(p);
    return dir;
}

Vector3d cloudMidpoint(PointCloud<PointXYZ>::ConstPtr cloud)
{
    PointXYZ minp, maxp;
    getMinMax3D(*cloud, minp, maxp);
    Vector3d minpt(minp.x, minp.y, minp.z);
    Vector3d maxpt(maxp.x, maxp.y, maxp.z);
    return (minpt + maxpt)/2;
}
Vector3d cloudMidpoint(PointCloud<PointXYZ>::ConstPtr cloud1, PointCloud<PointXYZ>::ConstPtr cloud2)
{
    PointXYZ min1, min2, max1, max2;
    getMinMax3D(*cloud1, min1, max1);
    getMinMax3D(*cloud2, min2, max2);
    Vector3d minpt(
            min(min1.x, min2.x),
            min(min1.y, min2.y),
            min(min1.z, min2.z));
    Vector3d maxpt(
            max(max1.x, max2.x),
            max(max1.y, max2.y),
            max(max1.z, max2.z));
    return (minpt + maxpt)/2;
}
