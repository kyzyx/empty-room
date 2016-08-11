// ----------------------
// rectanglerenderer.h
//
// This file contains definitions for classes that work with geometry
// consisting of axis-aligned rectangles

#ifndef _RECTANGLE_RENDERER
#define _RECTANGLE_RENDERER
#include <vector>
#include <algorithm>
#include <cmath>
#include "roommodel.h"
namespace roommodel {
class Rect {
    public:
        Rect() {
            for (int i = 0; i < 3; ++i) {
                p[i] = 0;
            }
            axis = 1;
            w = h = 0;
			depth = 0.1;
        }
        Rect(const Rect& r) {
            for (int i = 0; i < 3; ++i) {
                p[i] = r.p[i];
            }
            axis = r.axis;
            w = r.w;
            h = r.h;
            normal = r.normal;
            material = r.material;
			depth = r.depth;
        }
        Rect(double x, double y, double z) {
            p[0] = x; p[1] = y; p[2] = z;
            axis = 1;
            w = h = 0;
			depth = 0.1;
		}
        Rect(double x1, double y1, double z1,
             double x2, double y2, double z2) {
            p[0] = std::min(x1,x2);
            p[1] = std::min(y1,y2);
            p[2] = std::min(z1,z2);

            if (x1 == x2) {
                axis = 0;
                w = std::abs(y1-y2);
                h = std::abs(z1-z2);
			} else if (y1 == y2) {
                axis = 1;
                w = std::abs(x1-x2);
                h = std::abs(z1-z2);
            } else {
                axis = 2;
                w = std::abs(x1-x2);
                h = std::abs(y1-y2);
            }
			depth = 0.1;
        }

        std::pair<int,int> axisIndices() {
            return std::make_pair(axis?0:1, axis==2?1:2);
        }

        double p[3]; // coordinates of corner
        double w, h; // dimensions of rectangle
        int axis;    // which axis is the constant one
        int normal;
		double depth;
        Material* material;
};

// Generates a list of rectangles consisting of the base rectangle with cut out
// geometry for the diff rectangles
void rectangleDiff(
        Rect base,
        std::vector<Rect>& diff,
        std::vector<Rect>& generated);

// Turns the rectangles into a list of triangle vertex coordinates
void rectanglesToTriangles(
        std::vector<Rect>& rectangles,
        std::vector<double>& triangles,
        bool includebackfaces=true,
        bool includenormals=false,
        bool includeuvs=false);
}
#endif
