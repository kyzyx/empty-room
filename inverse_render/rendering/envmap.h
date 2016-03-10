#ifndef _ENVMAP_H
#define _ENVMAP_H
#include <vector>
#include <utility>

// Environment map: forward (+x), right, back, left, top(back up), bottom (forward up)

void computeEnvmapAdjacencies(std::vector<std::pair<int, int> >& adj, int res);
int getEnvmapCell(double x, double y, double z, int res);
#endif
