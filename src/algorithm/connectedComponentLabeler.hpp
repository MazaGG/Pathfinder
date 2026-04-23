#pragma once
#include <vector>
#include "../helpers/struct.hpp"
using namespace std;

class ConnectedComponentLabeler {

    private:
        Grid grid;
        Grid clusterId; // 0 means no ID yet
        vector<Point> centers;
        int minClusterSize;
        int id;  // Clusters smaller than this are filtered out
        
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

        int countCluster(const int& id) {
            int count = 0;

            for (int y = 0; y < clusterId.height; y++) {
                for (int x = 0; x < clusterId.width; x++) {
                    if (clusterId.cells[y][x] == id) {
                        count++;
                    }
                }
            }

            return count;
        } 

        bool causesSplit(const pair<int,int>& cell, const int& id, vector<int,int>& occupiedNeighbors) {
            
            // UNFINISHED
            return false;
        }

        void findClusters(const Grid& grid, const vector<Obstacle>& obstacles) {
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

        void updateOccupancyMap(const Point& change) {
            // flip values
            grid.cells[(int)change.y][(int)change.x] = (grid.cells[(int)change.y][(int)change.x] + 1) % 2;
        }

        void updateClusterMap(const int& oldId, const int& newId) {
            for (int y = 0; y < clusterId.height; y++) {
                for (int x = 0; x < clusterId.width; x++) {
                    if (clusterId.cells[y][x] == oldId) {
                        clusterId.cells[y][x] = newId;
                    }
                }
            }
        }

        // no need to optimize, this is out of scope and covered by hardware of scanner
        void updateClusterMap(const vector<Point>& changes) {
            vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

            for (int i = 0; i < changes.size(); i++) {
                updateOccupancyMap(changes[i]);
                int x = (int)changes[i].x;
                int y = (int)changes[i].y;
                int currentId = clusterId.cells[y][x];

                if (currentId == 0) { // new obs
                    // need to handle:
                    // 1. changes to cluster center when the scope of a cluster is increase
                    // 2. changes to cluster id assignment when the new obs merges 2 or more clusters into 1
                    bool isolated = true;
                    bool merge = false;
                    vector<int> mergeId; // keeps track of what cluster Id to merge to

                    // this function should determine how to handle
                    for (int j = 0; j < directions.size(); j++) {
                        pair<int,int> neighbor = {x + directions[j].first, y + directions[j].second};
                        
                        // check if out of bounds
                        if (neighbor.first < 0 || neighbor.first >= grid.width || neighbor.second < 0 || neighbor.second >= grid.height) {
                            continue;
                        }

                        int neighborId = clusterId.cells[neighbor.second][neighbor.first];
                        // need to check if all surrounding are 0
                        if (neighborId == 0) {
                            continue;
                        }
                        // else, keep track of the id, and check if it's consistent throughout
                        else {
                            if (mergeId.empty()) {
                                isolated = false;
                                mergeId.push_back(neighborId);
                            }
                            else { // means it's either adjacent to the same cluster or at least 2 different clusters
                                if (mergeId.size() == 1 && mergeId[0] != neighborId) {
                                    merge = true; // adjacent to multiple clusters and would need to merge them into one
                                }
                                if (merge) {
                                    mergeId.push_back(neighborId);
                                }
                            }
                        }
                    }

                    // the ff handles different possibilities
                    if (isolated) {
                        // give new Id and update cluster map
                        clusterId.cells[y][x] = id;
                        centers.push_back(findCenter(id));
                        id++;
                    }
                    else {
                        // combine it to the existing cluster it's adjacent to. update centers
                        if (!merge) {
                            Point center = findCenter(mergeId[0]);
                            clusterId.cells[y][x] = mergeId[0];

                            // update center in centers vector
                            for (int j = 0; j < centers.size(); j++) {
                                if (centers[j].x == center.x && centers[j].y == center.y) {
                                    centers[j] = findCenter(mergeId[0]);
                                    break;
                                }
                            }
                        }
                        // combine the clusters into one clusterId and update the cluster map (use the first detected mergeId)
                        else {
                            Point center = findCenter(mergeId[0]);
                            clusterId.cells[y][x] = mergeId[0];

                            for (int j = 1; j < mergeId.size(); j++) {
                                // need to remove from centers vector
                                Point remove = findCenter(mergeId[j]);
                                for (int k = 0; k < centers.size(); k++) {
                                    if (centers[k].x == remove.x && centers[k].y == remove.y) {
                                        centers.erase(centers.begin() + k);
                                        break;
                                    }
                                }

                                // need to update clusterId map
                                updateClusterMap(mergeId[j], mergeId[0]);
                            }

                            // update center in centers vector
                            for (int j = 0; j < centers.size(); j++) {
                                if (centers[j].x == center.x && centers[j].y == center.y) {
                                    centers[j] = findCenter(mergeId[0]);
                                    break;
                                }
                            }
                        }
                    }

                }

                else { // remove obs
                    // need to handle:
                    // 1. is isolated, remove from centers and update cluster map
                    // 2. if not isolated, check if its removal causes a split
                    //      2.1 if no, update the center and cluster map
                    //      2.2 if yes, do an expanding neighbor search for each possible neighbor
                    // NOTE: keep track of neighbor cells during BFS. If one of the scanned neighbors in BFS is an unchecked neighbor, pop it off the list to do a BFS scan
                    vector<pair<int,int>> occupiedNeighbors;

                    // this function should determine how to handle
                    for (int j = 0; j < directions.size(); j++) {
                        pair<int,int> neighbor = {x + directions[j].first, y + directions[j].second};
                        
                        // check if out of bounds
                        if (neighbor.first < 0 || neighbor.first >= grid.width || neighbor.second < 0 || neighbor.second >= grid.height) {
                            continue;
                        }

                        int neighborId = clusterId.cells[neighbor.second][neighbor.first];
                        // need to check if all surrounding are 0
                        if (neighborId == 0) {
                            continue;
                        }
                        // else, keep track of the id, and check if it's consistent throughout
                        else {
                            occupiedNeighbors.push_back(neighbor);
                        }
                    }

                    if (occupiedNeighbors.empty()) {
                        Point center = findCenter(currentId);
                        clusterId.cells[y][x] = 0;

                        // update center in centers vector
                        for (int j = 0; j < centers.size(); j++) {
                            if (centers[j].x == center.x && centers[j].y == center.y) {
                                centers[j] = findCenter(currentId);
                                break;
                            }
                        }
                    } else {
                        Point cneter = findCenter(currentId);
                        clusterId.cells[y][x] = 0;
                        // check if it causes a split by doing a count
                    }

                } 
            }
        }
        
    public:

        ConnectedComponentLabeler(Grid grid, vector<Obstacle>& obstacles, int minClusterSize = 1) {
            this->id = 0;
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

        void update(vector<Point> changes) {

        }
};