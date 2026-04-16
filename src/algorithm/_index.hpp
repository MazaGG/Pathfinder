#pragma once
#include <chrono>
#include "voronoiDiagram.hpp"
#include "pathfinder.hpp"
using namespace std;
using namespace chrono;

class HybridVoronoiA {

    private:
        vector<VoronoiVertex> graph;
        vector<Point> path;
        long time;
        double length;

    public:
        HybridVoronoiA(Grid& grid, vector<Obstacle>& obstacles, Point& start, Point& goal) {
            auto time_start = high_resolution_clock::now();
            VoronoiDiagram voronoiDiagram(obstacles, grid);
            this->graph = voronoiDiagram.getVertices();
            Pathfinder voronoiAstar(grid, graph, start, goal);
            this->path = voronoiAstar.findPath();
            auto time_end = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(time_end - time_start).count();
        }

        vector<VoronoiVertex> getGraph() {
            return graph;
        }

        vector<Point> getPath() {
            return path;
        }

        long getTime() {
            return time;
        }

        double getLength() {
            length = computePathLength(path);
            return length;
        }
};