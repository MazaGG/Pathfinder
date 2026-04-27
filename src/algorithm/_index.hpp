#pragma once
#include <chrono>
#include "voronoiDiagram.hpp"
#include "pathfinder.hpp"
#include "pathOptimization.hpp"
using namespace std;
using namespace chrono;

class HybridVoronoiA {

    private:
        vector<Point> path;
        vector<VoronoiVertex> vertices;
        double time;
        double length;

    public:
        HybridVoronoiA(Grid& grid, vector<Point>& centers, Point& start, Point& goal) {
            auto time_start = high_resolution_clock::now();
            VoronoiDiagram voronoiDiagram(grid, centers);
            this->vertices = voronoiDiagram.getVertices();
            Pathfinder voronoiAstar(grid, start, goal, vertices);
            vector<Point> temp_path = voronoiAstar.getPath();
            PathOptimization optimize(grid, temp_path);
            this->path = optimize.getPath();
            auto time_end = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(time_end - time_start).count();
            this->length = computePathLength(path);
        }

        vector<VoronoiVertex> getVertices() {
            return vertices;
        }

        vector<Point> getPath() {
            return path;
        }

        double getTime() {
            return time;
        }

        double getLength() {
            return length;
        }
};