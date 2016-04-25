#include "geometrygenerator.h"

namespace roommodel{
using namespace std;

void GeometryGenerator::generate() {
    double px = 0;
    double py = 0;
    double minx = 0;
    double maxx = 0;
    double miny = 0;
    double maxy = 0;
    double h = model->height;
    double delta = 0.02;

    for (int i = 0; i < model->walls.size(); ++i) {
        int nn = model->walls[i].normal;
        double nx = (i&1)?0:nn;
        double ny = (i&1)?nn:0;
        double l = model->walls[i].length*nn;
        double cx = (i&1)?px+l:px;
        double cy = (i&1)?py:py-l;
        if (cx < minx) minx = cx;
        if (cy < miny) miny = cy;
        if (cx > maxx) maxx = cx;
        if (cy > maxy) maxy = cy;
        Rect wall(px, py, -delta, cx, cy, h+delta);
        wall.normal = nn;
        wall.material = &(model->wallMaterial);
        vector<Rect> windowRects;
		for (int j = 0; j < model->walls[i].windows.size(); ++j) {
			RectangleWallObject& rwo = model->walls[i].windows[j];
			double coords[4][3]; // Coordinates of top left, top right, bottom left, bottom right of window
			coords[0][2] = rwo.verticalposition + rwo.height;
			coords[1][2] = coords[0][2];
			coords[2][2] = rwo.verticalposition;
			coords[3][2] = coords[2][2];
			if (i & 1) {
				for (int k = 0; k < 4; ++k) coords[k][1] = cy;
				coords[0][0] = px + nn*rwo.horizontalposition;
				coords[2][0] = coords[0][0];
				coords[1][0] = px + nn*(rwo.horizontalposition + rwo.width);
				coords[3][0] = coords[1][0];
			}
			else {
				for (int k = 0; k < 4; ++k) coords[k][0] = cx;
				coords[0][1] = py - nn*rwo.horizontalposition;
				coords[2][1] = coords[0][1];
				coords[1][1] = py - nn*(rwo.horizontalposition + rwo.width);
				coords[3][1] = coords[1][1];
			}
			// Add window geometry
			Rect wRect(coords[0][0], coords[0][1], coords[0][2],
				coords[3][0], coords[3][1], coords[3][2]);
			wRect.normal = wall.normal;
			windowRects.push_back(wRect);
			wRect.material = &(rwo.material);
			wRect.p[wall.axis] += wall.normal*rwo.recessed;
			windowRectangles[&(model->walls[i].windows[j])] = wRect;

			// Add trim geometry for windows
			Rect trect = wRect;
			trect.material = &(rwo.frameMaterial);
			trect.w += 2*rwo.trimWidth;
			trect.p[wall.axis] = wall.p[wall.axis] + trect.normal*rwo.trimDepth;
			trect.p[wall.axis ? 0 : 1] -= rwo.trimWidth;
			trect.p[2] -= rwo.trimWidth;
			trect.h = rwo.trimWidth;
			trect.depth = rwo.trimDepth;
			trimRectangles.push_back(trect);

			trect.p[2] += wRect.h+rwo.trimWidth;
			trimRectangles.push_back(trect);

			trect.h = wRect.h + 2*rwo.trimWidth;
			trect.w = rwo.trimWidth;
			trect.p[2] = wRect.p[2] - rwo.trimWidth;
			trimRectangles.push_back(trect);

			trect.p[trect.axis?0:1] += wRect.w+rwo.trimWidth;
			trimRectangles.push_back(trect);

			if (rwo.recessed != 0) {
				for (int k = 1; k <= 2; k ++)
					coords[k][wall.axis] += wall.normal*rwo.recessed;
				// Add recessed geometry for windows
				for (int n1 = 0; n1 < 4; n1 += 3) {
					for (int n2 = 1; n2 <= 2; n2++) {
						Rect edgeRect(coords[n1][0], coords[n1][1], coords[n1][2],
							coords[n2][0], coords[n2][1], coords[n2][2]);
						edgeRect.material = &(model->wallMaterial);
						edgeRect.normal = rwo.recessed<0?-1:1;
						if (n2 == 1) edgeRect.normal *= -1;
						if ((n1 + n2) % 2 == 0) edgeRect.normal *= wall.axis?-nn:nn;
						wallRectangles.push_back(edgeRect);
					}
				}
			}
        }
		vector<Rect> newRects;
		if (model->walls[i].windows.empty()) {
			newRects.push_back(wall);
		}
		else {
			rectangleDiff(wall, windowRects, newRects);
		}
		for (int j = 0; j < newRects.size(); ++j) {
			// Add baseboard geometry for relevant wall segments
			if (model->baseboardHeight > 0 && newRects[j].p[2] == -delta && newRects[j].h >= model->baseboardHeight) {
				Rect bbFront = newRects[j];
				bbFront.p[bbFront.axis ? 0 : 1] -= model->baseboardDepth;
				bbFront.p[bbFront.axis] += newRects[j].normal*model->baseboardDepth;
				bbFront.h = model->baseboardHeight;
				bbFront.w += 2 * model->baseboardDepth;
				bbFront.material = &(model->baseboardMaterial);
				bbFront.depth = model->baseboardDepth;
				baseboardRectangles.push_back(bbFront);
				/*
				Rect bbTop;
				if (newRects[j].normal > 0) bbTop = Rect(newRects[j].p[0], newRects[j].p[1], model->baseboardHeight);
				else                        bbTop = Rect(newRects[j].p[0] - newRects[j].w, newRects[j].p[1] - newRects[j].h, model->baseboardHeight);

				if (newRects[j].axis == 0) {
					if (newRects[j].normal > 0) bbTop = Rect(newRects[j].p[0], newRects[j].p[1] - model->baseboardDepth, model->baseboardHeight);
					else                        bbTop = Rect(newRects[j].p[0] - model->baseboardDepth, newRects[j].p[1] - model->baseboardDepth, model->baseboardHeight);
					bbTop.h = newRects[j].w+2*model->baseboardDepth;
					bbTop.w = model->baseboardDepth;
				}
				else if (newRects[j].axis == 1) {
					if (newRects[j].normal > 0) bbTop = Rect(newRects[j].p[0] - model->baseboardDepth, newRects[j].p[1], model->baseboardHeight);
					else                        bbTop = Rect(newRects[j].p[0] - model->baseboardDepth, newRects[j].p[1] - model->baseboardDepth, model->baseboardHeight);
					bbTop.w = newRects[j].w+2*model->baseboardDepth;
					bbTop.h = model->baseboardDepth;
				}
				bbTop.material = &(model->baseboardMaterial);
				bbTop.axis = 2;
				bbTop.normal = 1;
				baseboardRectangles.push_back(bbTop);
				*/
			}
		}
		wallRectangles.insert(wallRectangles.end(), newRects.begin(), newRects.end());
        px = cx;
        py = cy;
    }
    Rect floorRect(minx-delta, miny-delta, 0, maxx+delta, maxy+delta, 0);
    floorRect.material = &(model->floorMaterial);
    floorRect.normal = 1;
    floorRectangles.push_back(floorRect);
    Rect ceilRect(minx-delta, miny-delta, h, maxx+delta, maxy+delta, h);
    ceilRect.material = &(model->ceilingMaterial);
    ceilRect.normal = -1;
    ceilRectangles.push_back(ceilRect);
}

void GeometryGenerator::getRectangles(vector<Rect>& rectangles) {
	rectangles.insert(rectangles.end(), floorRectangles.begin(), floorRectangles.end());
	rectangles.insert(rectangles.end(), ceilRectangles.begin(), ceilRectangles.end());
	rectangles.insert(rectangles.end(), trimRectangles.begin(), trimRectangles.end());
	rectangles.insert(rectangles.end(), wallRectangles.begin(), wallRectangles.end());
	rectangles.insert(rectangles.end(), baseboardRectangles.begin(), baseboardRectangles.end());
}

void GeometryGenerator::getTriangleGeometry(vector<double>& triangles) {
	vector<Rect> rectangles;
	getRectangles(rectangles);
    rectanglesToTriangles(rectangles, triangles, false, true);
}
void GeometryGenerator::getTriangleVertexColors(std::vector<double>& colors) {
    vector<Rect> rectangles;
    getRectangles(rectangles);
    for (int i = 0; i < rectangles.size(); ++i) {
        for (int j = 0; j < 6; ++j) {
            colors.push_back(rectangles[i].material->diffuse.r);
            colors.push_back(rectangles[i].material->diffuse.g);
            colors.push_back(rectangles[i].material->diffuse.b);
        }
    }
}

Rect GeometryGenerator::getRectangleForWindow(RectangleWallObject* rwo) {
	return windowRectangles[rwo];
}
}
