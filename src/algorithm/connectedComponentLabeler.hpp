#pragma once
#include <vector>
#include "../helpers/struct.hpp"
using namespace std;

class ConnectedComponentLabeler {

    private:
        Grid grid;
        Grid clusterId; // 0 means no ID yet
        vector<Point> centers;
        int minClusterSize;  // Clusters smaller than this are filtered out
        
        Point findCenter(const int& id) {
            double sumX = 0.0;
            double sumY = 0.0;
            int count = 0;

            // scan the entire clusterId grid
            for (int y = 0; y < clusterId.height; y++) {
                for (int x = 0; x < clusterId.width; x++) {
                    if (clusterId.cells[y][x] == id) {
                        sumX += x;
                        sumY += y;
                        count++;
                    }
                }
            }

            return {sumX / count, sumY / count};
        }

        void findClusters(const Grid& grid, const vector<Obstacle>& obstacles) {
            int id = 1;
            vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

            for (int i = 0; i < obstacles.size(); i++) {
                if (obstacles[i].vertices.empty()) {
                    continue;
                }
                Point initial = obstacles[i].vertices[0];
                int initial_x = initial.x >= grid.width ? (int)(initial.x - 1) : (int)initial.x;
                int initial_y = initial.y >= grid.height ? (int)(initial.y - 1) : (int)initial.y;

                int& obsId = clusterId.cells[initial_y][initial_x];
                if (obsId != 0) {
                    continue; // this means that the obstacle is connected to another obstacle. Thus, it is not isolated and doesn't need an id.
                }                
                obsId = id;
                
                vector<pair<int,int>> neighbors;
                neighbors.push_back(pair<int,int>(initial_x, initial_y));

                while (!neighbors.empty()) {
                    pair<int,int> current = neighbors.back();
                    neighbors.pop_back();
                    // add neighbors to the queue
                    for (int j = 0; j < directions.size(); j++) {
                        pair<int,int> neighbor = {current.first + directions[j].first, current.second + directions[j].second};
                        // check if out of bounds
                        if (neighbor.first < 0 || neighbor.first >= grid.width || neighbor.second < 0 || neighbor.second >= grid.height) {
                            continue;
                        }
                        // check if already visited
                        if (clusterId.cells[neighbor.second][neighbor.first] == id) {
                            continue;
                        }
                        // check if occupied
                        if (grid.cells[neighbor.second][neighbor.first] == 1) {
                            clusterId.cells[neighbor.second][neighbor.first] = id;
                            neighbors.push_back(neighbor);
                        }
                    }
                }

                centers.push_back(findCenter(id));
                id++;
            }
        }
        
    public:

        ConnectedComponentLabeler(Grid grid, vector<Obstacle>& obstacles, int minClusterSize = 1) {
            this->grid = grid;
            this->clusterId = {grid.width, grid.height, vector(grid.height, vector<int>(grid.width, 0))};
            findClusters(grid, obstacles);
            this->minClusterSize = minClusterSize;
        };
        
        vector<Point> getCenters() {
            return centers;
        }

        Grid getClusters() {
            return clusterId;
        }
};