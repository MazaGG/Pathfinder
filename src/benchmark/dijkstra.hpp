#pragma once
#include <vector>
#include <chrono>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
using namespace std;
using namespace chrono;

class Dijkstra {

    private:
        vector<Point> path;
        vector<vector<DJKNode>> bfsGrid;
        vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
        double time;
        double length;

        void findPath(const Grid& grid, const Point& start, const Point& goal) {
            priority_queue<DJKNode, vector<DJKNode>, greater<DJKNode>> openList;
            bfsGrid[(int)start.y][(int)start.x].x = (int)start.x;
            bfsGrid[(int)start.y][(int)start.x].y = (int)start.y;
            bfsGrid[(int)start.y][(int)start.x].gscore = 0;
            openList.push(bfsGrid[(int)start.y][(int)start.x]);
            double moveCost = 1;
            bool goalReached = false;
            pair<int,int> endIndex = {-1,-1};

            while (!openList.empty()) {
                DJKNode current = openList.top();
                openList.pop();

                if (bfsGrid[current.y][current.x].isClosed) {
                    continue;
                }
                bfsGrid[current.y][current.x].isClosed = true;

                if (current.x == (int)goal.x && current.y == (int)goal.y) {
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
                    if (gscore < bfsGrid[y][x].gscore) {
                        bfsGrid[y][x].x = x;
                        bfsGrid[y][x].y = y;
                        bfsGrid[y][x].gscore = gscore;
                        bfsGrid[y][x].parentX = current.x;
                        bfsGrid[y][x].parentY = current.y;
                        openList.push(bfsGrid[y][x]);
                    }
                }
            }

            if (goalReached) {
                int x = endIndex.first;
                int y = endIndex.second;
                while (x != -1 && y != -1) {
                    path.push_back({(double)x, (double)y});
                    int next_x = bfsGrid[y][x].parentX;
                    int next_y = bfsGrid[y][x].parentY;
                    x = next_x;
                    y = next_y;
                }
            }

            reverse(path.begin(), path.end());
            path.push_back(goal);
        }
    
    public:

        Dijkstra(Grid& grid, Point& start, Point& goal) {
            auto start_time = high_resolution_clock::now();
            this->bfsGrid.resize(grid.height, vector<DJKNode>(grid.width));
            findPath(grid, start, goal);
            auto end_time = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(end_time - start_time).count();
            this->length = computePathLength(path);
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