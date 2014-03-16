#ifndef _RERENDER_H
#define _RERENDER_H
#include "wall_finder.h"
#include "mesh.h"
#include "solver.h"

#include <string>

void outputRadianceFile(std::string filename, WallFinder& wf, Mesh& m, InverseRender& ir);
#endif
