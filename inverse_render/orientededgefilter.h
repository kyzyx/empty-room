#ifndef _ORIENTED_EDGE_FILTER_H
#define _ORIENTED_EDGE_FILTER_H
#include "datamanager/camparams.h"
#include  <vector>
#include <Eigen/Eigen>
void findVanishingPoints(
    const CameraParams& cam,
    R4Matrix normalization,
    std::vector<Eigen::Vector3d>& vps);
void orientedEdgeFilterVP(
    const char* image,
    float* edges,
    int w, int h,
    std::vector<Eigen::Vector3d>& vps,
    int windowy=21, int windowx=5);
void createEdgeImage(const CameraParams* cam, const void* colorimage, void* image);
#endif
