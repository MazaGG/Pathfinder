#pragma once
#include <vector>
#include <queue>
#include <chrono>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include "voronoiDiagram.hpp"
using namespace std;

class Pathfinder {

    private:
        // vector<Point> globalPath;
        // vector<Point> localPath;
        int searchId;
        vector<Point> path;
        vector<vector<AstarNode>> astarGrid;
        vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

        Point getTraversable(const Grid& grid, const Point& p, const Point& q) { // using Bresenham's line algorithm
            int x0 = (int)p.x;
            int y0 = (int)p.y;
            int x1 = (int)q.x;
            int y1 = (int)q.y;
            int dx = abs(x1 - x0);
            int dy = abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1;
            int sy = (y0 < y1) ? 1 : -1;
            int err = dx - dy;

            Point lastValidPoint = p;
            while (true) {

                if (grid.cells[y0][x0] == 1) {
                    return lastValidPoint;
                }

                lastValidPoint = Point{(double)x0, (double)y0};

                if (x0 == x1 && y0 == y1) {
                    return q;
                }

                int e2 = 2 * err;
                if (e2 > -dy) {
                    err -= dy;
                    x0 += sx;
                }
                if (e2 < dx) {
                    err += dx;
                    y0 += sy;
                }
            }
        }

        Point findNearestFreeCell(const Grid& grid, const Point& start, const Point& goal) {
            searchId++;
            priority_queue<AstarNode, vector<AstarNode>, greater<AstarNode>> openList;

            int start_x = (int)start.x;
            int start_y = (int)start.y;
            AstarNode& startNode = astarGrid[start_y][start_x];
            startNode.x = start_x;
            startNode.y = start_y;
            startNode.gscore = 0;
            startNode.gscoreId = searchId;
            startNode.score = distance(start, goal);
            startNode.parentX = -1;
            startNode.parentY = -1;
            openList.push(startNode);
            double moveCost = 1;

            while(!openList.empty()) {
                AstarNode current = openList.top();
                openList.pop();

                if (astarGrid[current.y][current.x].closedId == searchId) {
                    continue;
                }
                astarGrid[current.y][current.x].closedId = searchId;

                if (grid.cells[current.y][current.x] == 0) {
                    return Point{(double)current.x, (double)current.y};
                }

                for (int i = 0; i < directions.size(); i++) {
                    int x = current.x + directions[i].first;
                    int y = current.y + directions[i].second;
                    moveCost = 1;

                    if (x < 0 || x >= grid.width || y < 0 || y >= grid.height) {
                        continue;
                    }
                    
                    if (directions[i].first != 0 && directions[i].second != 0) {
                        if (grid.cells[current.y][current.x + directions[i].first] == 1 && grid.cells[current.y + directions[i].second][current.x] == 1) {
                            continue;
                        }
                        moveCost = 1.414;
                    }

                    double gscore = current.gscore + moveCost;
                    AstarNode& neighbor = astarGrid[y][x];
                    if (neighbor.gscoreId != searchId || gscore < neighbor.gscore) {
                        neighbor.x = x;
                        neighbor.y = y;
                        neighbor.gscore = gscore;
                        neighbor.gscoreId = searchId;
                        neighbor.score = gscore + distance({(double)x, (double)y}, goal);
                        neighbor.parentX = current.x;
                        neighbor.parentY = current.y;
                        openList.push(neighbor);
                    }
                }
            }

            return Point{-1, -1};
        }

        void findPath(const Grid& grid, const Point& start, const Point& goal, vector<VoronoiVertex>& vertices) {
            int start_index = -1;
            int goal_index = -1;
            double bestStart = numeric_limits<double>::infinity();
            double bestGoal = numeric_limits<double>::infinity();

            // find point closes to start and goal
            for (int i = 0; i < vertices.size(); i++) {
                double startDistance = distance(start, vertices[i].position);
                double goalDistance = distance(goal, vertices[i].position);
                if (startDistance < bestStart) {
                    bestStart = startDistance;
                    start_index = i;
                }
                if (goalDistance < bestGoal) {
                    bestGoal = goalDistance;
                    goal_index = i;
                }
            }

            // Refactor: prolly need to so astar here when checking if start can go directly to start_index
            int endIndex = -1;
            double bestDistanceToGoal = distance(start, goal);
            priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> openList;
            vertices[start_index].gscore = distance(start, vertices[start_index].position);
            vertices[start_index].score = vertices[start_index].gscore + distance(goal, vertices[start_index].position);
            openList.push({vertices[start_index].score, start_index});

            while (!openList.empty()) {
                int currentIndex = openList.top().second;
                VoronoiVertex& current = vertices[currentIndex];
                openList.pop();

                if (current.isClosed) { // if already visited, skip
                    continue;
                }
                current.isClosed = true;

                // check if can traverse directly to goal
                if (isEdgeValid(grid, current.position, goal)) {
                    endIndex = current.index;
                    break;
                }

                // track node closest to goal
                double distanceToGoal = distance(current.position, goal);
                if (distanceToGoal < bestDistanceToGoal) {
                    bestDistanceToGoal = distanceToGoal;
                    endIndex = current.index;
                }

                // add neighbors
                for (int i = 0; i < current.neighbors.size(); i++) {
                    int& neighborIndex = current.neighbors[i];
                    VoronoiVertex& neighbor = vertices[neighborIndex];
                    double gscore = current.gscore + distance(current.position, neighbor.position);
                    double score = gscore + distance(current.position, goal);
                    if (gscore < neighbor.gscore) {
                        neighbor.gscore = gscore;
                        neighbor.score = score;
                        neighbor.parentIndex = current.index;
                        openList.push({neighbor.score, neighborIndex});
                    }
                }
            }

            buildPath(grid, endIndex, start, goal, vertices);
        }

        void buildPath (const Grid& grid, const int& endIndex, const Point& start, const Point& goal, const vector<VoronoiVertex>& vertices) {
            path.push_back(goal);

            for (int i = endIndex; i != -1; i = vertices[i].parentIndex) {
                Point nextPoint = vertices[i].position;

                if (grid.cells[nextPoint.y][nextPoint.x] == 1) {
                    nextPoint = findNearestFreeCell(grid, nextPoint, goal);
                }                

                Point segment_end = getTraversable(grid, nextPoint, path.back());
                
                if (segment_end == path.back()) {
                    path.push_back(nextPoint);
                }
                else {
                    // Refactor: it's possible that the voronoi vertex itself is inside the obstacle, and in some cases, the whole edge could be inside the obstacle
                    astar(grid, segment_end, path.back(), path);
                    path.push_back(nextPoint);
                }
            }
            
            reverse(path.begin(), path.end());
        }

        void astar(const Grid& grid, const Point& start, const Point& goal, vector<Point>& path) {
            searchId++;
            priority_queue<AstarNode, vector<AstarNode>, greater<AstarNode>> openList;

            int start_x = (int)start.x;
            int start_y = (int)start.y;
            AstarNode& startNode = astarGrid[start_y][start_x];
            startNode.x = start_x;
            startNode.y = start_y;
            startNode.gscore = 0;
            startNode.gscoreId = searchId;
            startNode.score = distance(start, goal);
            startNode.parentX = -1;
            startNode.parentY = -1;
            openList.push(startNode);

            double moveCost = 1;
            bool goalReached = false;
            pair<int,int> endIndex = {-1,-1};

            while (!openList.empty()) {
                AstarNode current = openList.top();
                openList.pop();

                if (astarGrid[current.y][current.x].closedId == searchId) {
                    continue;
                }
                astarGrid[current.y][current.x].closedId = searchId;

                if (isEdgeValid(grid, Point{(double)current.x, (double)current.y}, goal)) {
                    goalReached = true;
                    endIndex = {current.x, current.y};
                    break;
                }

                for (int i = 0; i < directions.size(); i++) {
                    int x = current.x + directions[i].first;
                    int y = current.y + directions[i].second;
                    moveCost = 1;

                    if (x < 0 || x >= grid.width || y < 0 || y >= grid.height) {
                        continue;
                    }

                    if (grid.cells[y][x] == 1) {
                        continue;
                    }

                    if (directions[i].first != 0 && directions[i].second != 0) {
                        if (grid.cells[current.y][current.x + directions[i].first] == 1 && grid.cells[current.y + directions[i].second][current.x] == 1) {
                            continue;
                        }
                        moveCost = 1.414;
                    }

                    double gscore = current.gscore + moveCost;
                    AstarNode& neighbor = astarGrid[y][x];
                    if (neighbor.gscoreId != searchId || gscore < neighbor.gscore) {
                        neighbor.x = x;
                        neighbor.y = y;
                        neighbor.gscore = gscore;
                        neighbor.gscoreId = searchId;
                        neighbor.score = gscore + distance({(double)x, (double)y}, goal);
                        neighbor.parentX = current.x;
                        neighbor.parentY = current.y;
                        openList.push(neighbor);
                    }
                }
            }

            if (goalReached) {
                int x = endIndex.first;
                int y = endIndex.second;
                while (x != -1 && y != -1) {
                    path.push_back({(double)x, (double)y});
                    int next_x = astarGrid[y][x].parentX;
                    int next_y = astarGrid[y][x].parentY;
                    x = next_x;
                    y = next_y;
                }
            }
        }
    
    public:
        Pathfinder(Grid& grid, Point& start, Point& goal, vector<VoronoiVertex> vertices) {
            this->searchId = -1;
            this->astarGrid.resize(grid.height, vector<AstarNode>(grid.width));
            findPath(grid, start, goal, vertices);
        }

        vector<Point> getPath() {
            return path;
        }
    
};