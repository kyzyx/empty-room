#ifndef _ROOMMODEL_H
#define _ROOMMODEL_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <map>

class RoomModel {
    public:
        RoomModel() {}
        ~RoomModel() {}

        void setAxes(Eigen::Vector3d a, Eigen::Vector3d b);
        void addCloud(
                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                std::vector<Eigen::Vector4d>& planes,
                std::vector<bool>& alignedto,
                Eigen::Matrix4d transform,
                double rmse
        );
        void closeLoop(
                Eigen::Matrix4d transform,
                double rmse
        );

        Eigen::Matrix4d getTransform(int n) { return xforms[n]; }
        Eigen::Matrix4d getCumulativeTransform(int n) { return cumxforms[n]; }

    private:
        void distributeRotation(Eigen::Vector4d plane, int frame);
        void distributeTranslation(int orientation, double translation, int endframe, int startframe, double residualweight=0);
        void checkLoopClosure(Eigen::Vector4d plane, int frame);
        void recomputeCumulativeTransforms(int start=0);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > clouds;
        std::vector<Eigen::Matrix4d> xforms;
        std::vector<Eigen::Matrix4d> adjustments;
        std::vector<double> rotationangles;
        std::vector<int> rotationaxes;

        std::vector<Eigen::Matrix4d> cumxforms;
        std::vector<std::vector<Eigen::Vector4d> > allplanes;
        std::vector<double> weights;
        std::map<double, int> roomplanes[6];
        std::vector<Eigen::Vector3d> axes;
        std::vector<int> constrained[3];
        std::vector<bool> seen[3];
};

#endif
