#ifndef _RECTIFY_H
#define _RECTIFY_H
#include <Eigen/Eigen>
#include <vector>
#include "R2ImageOld.h"

Eigen::Matrix3d getRectificationMatrix(std::vector<Eigen::Vector2d>& a, std::vector<Eigen::Vector2d>& b);
void rectify(Eigen::Matrix3d m, R2Image_Old::R2Image* image);

#endif
