#pragma once
#include <vector>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include "voronoiDiagram.hpp"
using namespace std;

class Pathfinder {

    private:
        vector<Point> path;
        vector<Point> globalPath;
        vector<Point> localPath;

    
    public:
        Pathfinder(Delaunay& graph, Grid& grid, Point& start, Point& goal) {
            // find global path
            // if global path is empty, return no path exists
            // traverse global path. if collision, find local path to bypass the obstacle and merge with global path after bypassing the obstacle.
        }

        vector<Point> getPath() {
            return path;
        }
    
};