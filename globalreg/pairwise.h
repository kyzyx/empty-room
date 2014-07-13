#ifndef _PAIRWISE_H
#define _PAIRWISE_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class AlignmentResult {
    public:
        AlignmentResult(Eigen::Matrix4d xform) : transform(xform), error(0) {}
        AlignmentResult(Eigen::Matrix4d xform, double err) : transform(xform), error(err) {}
        AlignmentResult& operator=(const AlignmentResult& ar) {
            if (this != &ar) {
                error = ar.error;
                transform = ar.transform;
            }
            return *this;
        }
        double error;
        Eigen::Matrix4d transform;
};

/**
 * Calculates the transformation bringing plane src coplanar to plane tgt
 */
Eigen::Matrix4d overlapPlanes(Eigen::Vector4d src, Eigen::Vector4d tgt);
Eigen::Matrix4d overlapEdge(Eigen::Vector4d src1, Eigen::Vector4d src2, Eigen::Vector4d tgt1, Eigen::Vector4d tgt2);
Eigen::Matrix4d overlapCorner(
        Eigen::Vector4d src1, Eigen::Vector4d src2, Eigen::Vector4d src3,
        Eigen::Vector4d tgt1, Eigen::Vector4d tgt2, Eigen::Vector4d tgt3);

/**
 * Calculates which planes in source map to which planes in tgt. Returns the
 * number of correspondences found
 */
int findPlaneCorrespondences(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids,
        std::vector<int>& planecorrespondences);

/**
 * Returns the transformation mapping src to tgt
 * Assumes that the planes in tgt and src have already been computed in the
 * appropriate planes and ids vectors, or it will calculate them.
 *
 * Does NOT transform src or srcplanes into tgt's coordinate system.
 */
AlignmentResult align(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids);

/**
 * As align(...), but specifically forces the first planes in correspondence to be coplanar.
 */
AlignmentResult alignPlaneToPlane(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids,
        std::vector<int>& planecorrespondences,
        int maxiterations);

AlignmentResult alignEdgeToEdge(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids,
        std::vector<int>& planecorrespondences,
        int maxiterations);

AlignmentResult alignCornerToCorner(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids,
        std::vector<int>& planecorrespondences);

AlignmentResult alignICP(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr src,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr tgt,
        std::vector<Eigen::Vector4d>& srcplanes, std::vector<int>& srcids,
        std::vector<Eigen::Vector4d>& tgtplanes, std::vector<int>& tgtids,
        std::vector<int>& planecorrespondences,
        int maxiterations);
/**
 * Applies the transform to the plane in Ax + By + Cz + D = 0 form. Assumes the
 * normal (A,B,C) is normalized.
 */
Eigen::Vector4d transformPlane(Eigen::Vector4d plane, Eigen::Matrix4d transform);
#endif
