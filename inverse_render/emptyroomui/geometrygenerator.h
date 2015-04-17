// ----------------------
// geometrygenerator.h
//
// This file contains the class definition for the geometry generator class.
// It converts a room model into a set of rectangles to be rendered.

#ifndef _GEOMETRY_GENERATOR
#define _GEOMETRY_GENERATOR
#include "roommodel.h"
#include "rectanglerenderer.h"
#include <map>
namespace roommodel {
class GeometryGenerator {
    public:
        GeometryGenerator(RoomModel* room) : model(room) { ; }

        void generate();
        void getRectangles(std::vector<Rect>& rectangles);
        void getTriangleGeometry(std::vector<double>& triangles);
        void getTriangleVertexColors(std::vector<double>& colors);
		Rect getRectangleForWindow(RectangleWallObject* rwo);
    protected:
        RoomModel* model;

		std::map<RectangleWallObject*, Rect> windowRectangles;
        std::vector<Rect> wallRectangles;
        std::vector<Rect> baseboardRectangles;
        std::vector<Rect> otherRectangles;
};
}
#endif
