#pragma once
#include <vector>
#include <limits>
#include <queue>
#include <cmath>
#include <chrono>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp" 
using namespace std;
using namespace chrono;

class Pathfinder {

    private:
        Grid grid;
        vector<VoronoiVertex> graph;
        vector<bool> visited;
        Point start;
        Point goal;

        void backtrackVoronoi(vector<Point>& path, int& goalIdx, vector<VoronoiVertex>& graph, vector<int>& parent) {
            // cout << "V Path: ";
            for (int i = goalIdx; i != -1; i = parent[i]) {
                path.push_back(graph[i].position);
                visited[i] = true;
                // cout << "(" << graph[i].position.x << "," << graph[i].position.y << ")";
            }
            // cout << "\n";
            reverse(path.begin(), path.end());
        }

        vector<Point> backtrackAstar(ANode& current, vector<vector<AstarCell>>& gridData) {
            vector<Point> path;
            // cout << "A Path: ";
            while (current.x != -1 && current.y != -1) {
                path.push_back({(double)current.x, (double)current.y});
                pair<int,int> parent = gridData[current.y][current.x].parent;
                // cout << "(" << current.x << "," << current.y << ")";
                current.x = parent.first;
                current.y = parent.second;
            }
            // cout << "\n";
            reverse(path.begin(),path.end());
            return path;
        }

        vector<Point> voronoiAstar(Point& start, int& currentIdx, vector<bool>& visited) {
            vector<Point> path;
            priority_queue<VNode, vector<VNode>, greater<VNode>> open;
            vector<bool>& closed = visited;
            vector<double> gScore(graph.size(), numeric_limits<double>::infinity());
            vector<int> parent(graph.size(), -1);
            
            // find the voronoi vertex closes to the goal
            int goalIdx = -1;
            double bestDistance = numeric_limits<double>::infinity();
            for (int i = 0; i < graph.size(); i++) {
                double testDist = distance(graph[i].position, goal);
                if (testDist < bestDistance) {
                    bestDistance = testDist;
                    goalIdx = i;
                }
            }

            gScore[currentIdx] = 0;
            open.push({currentIdx, distance(graph[currentIdx].position, goal)});
            int bestToGoalIdx = currentIdx;
            double bestToGoal = distance(graph[currentIdx].position, goal);

            while (!open.empty()) {
                VNode current = open.top();
                open.pop();
        
                // ERROR: possible source of error ?
                currentIdx = current.idx;
                if (closed[currentIdx]) {
                    continue;
                }
                closed[currentIdx] = true;

                // keeps track of the best path so far
                double distToGoal = distance(graph[currentIdx].position, goal);
                if (distToGoal < bestToGoal) {
                    bestToGoal = distToGoal;
                    bestToGoalIdx = currentIdx;
                }

                // goal found
                if (currentIdx == goalIdx) {
                    bestToGoalIdx = currentIdx;
                    break;
                }

                // add valid neighbors to the open list and update gscore accordingly
                for (int i = 0; i < graph[currentIdx].neighbors.size(); i++) {
                    int neighborIdx = graph[currentIdx].neighbors[i];
                    if (!isEdgeValid(grid, graph[currentIdx].position, graph[neighborIdx].position)) {
                        continue;
                    }
                    
                    double cost = gScore[currentIdx] + distance(graph[currentIdx].position, graph[neighborIdx].position);
                    if (cost < gScore[neighborIdx]) {
                        gScore[neighborIdx] = cost;
                        parent[neighborIdx] = currentIdx;
                        double score = cost + distance(graph[neighborIdx].position, graph[goalIdx].position);
                        open.push({neighborIdx, score});
                    }
                }
            }
            
            // int startIdx = bestToGoalIdx;
            // check if the start can connect to the "best start" in voronoi graph
            // cout << "Start in vor: " << path[0].x << "," << path[0].y << "\n";
            // if (!isEdgeValid(grid, start, {path[0].x, path[0].y})) {
            //     cout << "ENTERS THIS because start is: " << start.x << "," << start.y << "\n";
            //     vector<Point> start_path = astarGrid(start, startIdx);
            //     path.insert(path.begin(), start_path.begin(), start_path.end() - 1);
            // }
            currentIdx = bestToGoalIdx;
            backtrackVoronoi(path, bestToGoalIdx, graph, parent);
            // cout << "bestIdx: " << currentIdx << "\n";
            return path;
        }

        vector<Point> astarGrid(const Point start, int& v_entry) {
            vector<Point> path;
            vector<vector<AstarCell>> gridData(grid.height, vector<AstarCell>(grid.width, AstarCell{false, numeric_limits<double>::infinity(), pair<int,int>(-1,-1)}));
            priority_queue<ANode, vector<ANode>, greater<ANode>> open;
            bool directToGoal = false;
            int start_x = (int)start.x;
            int start_y = (int)start.y;
            int goal_x = (int)goal.x;
            int goal_y = (int)goal.y;
            ANode current;
            gridData[start_y][start_x].gScore = 0;
            open.push({start_x, start_y, manhattan(start_x, start_y, goal_x, goal_y)});

            //directions
            static const vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1}};

            while (!open.empty()) {
                current = open.top();
                open.pop();

                // check if can go to goal directly
                if (isEdgeValid(grid, {(double)current.x, (double)current.y}, goal)) {
                    directToGoal = true;
                    break;
                }

                if (gridData[current.y][current.x].isClose) {
                    continue;
                }
                gridData[current.y][current.x].isClose = true;

                // goal found
                if (current.x == goal_x && current.y == goal_y) {
                    break;
                }

                // check if can connect to a voronoi vertex
                // v_entry = findNearestVisibleVertex({(double)current.x, (double)current.y}, goal, graph, visited, grid);
                // if (v_entry != -1) {
                //     cout << "Crit Point: " << current.x << "," << current.y << "\n";
                //     cout << "V_entry: " << graph[v_entry].position.x << "," << graph[v_entry].position.y << "\n";
                //     path.push_back(graph[v_entry].position);
                //     break;
                // }

                // add valid neighbors to the open list and update gscore accordingly
                for (int i = 0; i < directions.size(); i++) {
                    int neighbor_x = current.x + directions[i].first;
                    int neighbor_y = current.y + directions[i].second;

                    if (neighbor_x < 0 || neighbor_x >= grid.width || neighbor_y < 0 || neighbor_y >= grid.height) {
                        continue;
                    }
                    if (grid.cells[neighbor_y][neighbor_x] == 1) {
                        continue;
                    }

                    double gScore = gridData[current.y][current.x].gScore + manhattan(current.x, current.y, neighbor_x, neighbor_y);
                    if (gScore < gridData[neighbor_y][neighbor_x].gScore) {
                        gridData[neighbor_y][neighbor_x].gScore = gScore;
                        gridData[neighbor_y][neighbor_x].parent = pair<int,int>(current.x, current.y);
                        double score = gScore + manhattan(neighbor_x, neighbor_y, goal_x, goal_y);
                        open.push({neighbor_x, neighbor_y, score});
                    }
                }
            }

            vector<Point> backtrack = backtrackAstar(current, gridData);
            if (v_entry != -1) {
                backtrack.push_back(graph[v_entry].position);
            }
            if (directToGoal) {
                backtrack.push_back(goal);
            }
            return backtrack;
        }

        // JPS pseudocode
        // - fixed yung pagcheck mo for each direction (eg., for horizontal, horizontal lang dapat yung best move)
        // - due to obstacles, you will find forced neighbors (the goal is also considered a force neighbor)
        // - the parent of the forced neighbors / the node with a forced neighbor becomes a jump point and gets added to the open list
        // - do A star algorithm normally
        // void jumpPointSearch(Grid& grid, vector<VoronoiVertex>& graph, Point& start, Point& goal) {

        // }
    
    public:
        
        Pathfinder(Grid grid, vector<VoronoiVertex> graph, Point start, Point goal) {
            this->grid = grid;
            this->graph = graph;
            this->start = start;
            this->goal = goal;
            this->visited = vector<bool>(graph.size(), false);
        }
        
        vector<Point> findPath() {
            vector<Point> path;
            path.push_back(start);
            Point current = start;
            // check if can go directly to goal
            if (isEdgeValid(grid, start, goal)) {
                path.push_back(goal);
                return path;
            }
            // otherwise, trigger hybrid pathfinder
            int v_entry = findNearestVisibleVertex(current, goal, graph, visited, grid);
            int a_entry = -1;
            while(true) { 
                // cout << "TOP CURRENT: " << current.x << "," << current.y << "\n";
                // cout << "TOP GOAL: " << goal.x << "," << goal.y << "\n";
                if (v_entry != -1 && a_entry == -1) {
                    // auto vstar_start = high_resolution_clock::now();
                    // cout << "v_entry: " << v_entry << "\n";
                    // cout << "v: " << graph[v_entry].position.x << "," << graph[v_entry].position.y << "\n";
                    vector<Point> v_path = voronoiAstar(path.back(), v_entry, visited);
                    current = v_path.back();
                    // cout << "First vertex from vor: " << v_path[0].x << "," << v_path[0].y << "\n";
                    // cout << "Last vertex from vor: " << graph[v_entry].position.x << "," << graph[v_entry].position.y << "\n";
                    a_entry = v_entry;
                    path.insert(path.end(), v_path.begin() + 1, v_path.end());
                    // check if can go directly to the voronoi vertex from starting node:
                    // auto vstar_end = high_resolution_clock::now();
                    // cout << "voronoiAstar time: " << duration_cast<milliseconds>(vstar_end - vstar_start).count() << "ms\n";
                } 
                else if (a_entry != -1) {
                    // auto astar_start = high_resolution_clock::now();
                    // cout << "v_entry: " << v_entry << "\n";
                    // cout << "a_entry: " << a_entry << "\n";
                    // cout << "a: " << graph[a_entry].position.x << "," << graph[a_entry].position.y << "\n";
                    vector<Point> a_path = astarGrid(current, a_entry);
                    current = a_path.back();
                    a_entry = -1;
                    path.insert(path.end(), a_path.begin() + 1, a_path.end());
                    // auto astar_end = high_resolution_clock::now();
                    // cout << "astarGrid time: " << duration_cast<milliseconds>(astar_end - astar_start).count() << "ms\n";
                }
                else {
                    break;
                }
                if (path.back().x == goal.x && path.back().y == goal.y) {
                    break;
                }
            }

            return path;
        }
};