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
        vector<vector<AstarNode>> astarGrid;
        int searchId;
        double time;
        double length;

    public:
        HybridVoronoiA (const Grid& grid) {
            this->searchId = 0;
            this->astarGrid.resize(grid.height, vector<AstarNode>(grid.width));
        };

        void run(Grid& grid, vector<Point>& centers, Point& start, Point& goal) {
            auto time_start = high_resolution_clock::now();
            VoronoiDiagram voronoiDiagram(grid, centers);
            this->vertices = voronoiDiagram.getVertices();
            auto pathfinder_start = high_resolution_clock::now();
            Pathfinder voronoiAstar(grid, astarGrid, searchId, start, goal, vertices);
            // this->path = voronoiAstar.getPath();
            vector<Point> temp_path = voronoiAstar.getPath();
            auto optimize_start = high_resolution_clock::now();
            PathOptimization optimize(grid, temp_path);
            this->path = optimize.getPath();
            auto time_end = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(time_end - time_start).count();
            this->length = computePathLength(path);
            cout << "Voronoi: " << duration_cast<milliseconds>(pathfinder_start - time_start).count() << "ms\n";
            cout << "Pathfinder: " << duration_cast<milliseconds>(optimize_start - pathfinder_start).count() << "ms\n";
            cout << "Optimization: " << duration_cast<milliseconds>(time_end - optimize_start).count() << "ms\n";
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