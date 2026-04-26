#pragma once
#include <vector>
#include <chrono>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

struct Node {
    int x, y;
    int parentX = -1;
    int parentY = -1;
    int isClosed = false;
    double gscore = numeric_limits<double>::infinity();
    double score = numeric_limits<double>::infinity();

    bool operator > (const Node& other) const {
        return score > other.score;
    }        
};

class Astar {

    private:
        vector<Point> path;
        vector<vector<Node>> astarGrid;
        vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

        void findPath(const Grid& grid, const Point& start, const Point& goal) {
            priority_queue<Node, vector<Node>, greater<Node>> openList;
            astarGrid[(int)start.y][(int)start.x].x = (int)start.x;
            astarGrid[(int)start.y][(int)start.x].y = (int)start.y;
            astarGrid[(int)start.y][(int)start.x].gscore = 0;
            openList.push(astarGrid[(int)start.y][(int)start.x]);
            double moveCost = 1;
            bool goalReached = false;
            pair<int,int> endIndex = {-1,-1};

            while (!openList.empty()) {
                Node current = openList.top();
                openList.pop();

                if (astarGrid[current.y][current.x].isClosed) {
                    continue;
                }
                astarGrid[current.y][current.x].isClosed = true;

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
                    if (gscore < astarGrid[y][x].gscore) {
                        astarGrid[y][x].x = x;
                        astarGrid[y][x].y = y;
                        astarGrid[y][x].gscore = gscore;
                        astarGrid[y][x].score = gscore + distance({(double)x, (double)y}, goal);
                        astarGrid[y][x].parentX = current.x;
                        astarGrid[y][x].parentY = current.y;
                        openList.push(astarGrid[y][x]);
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

            reverse(path.begin(), path.end());
            path.push_back(goal);
        }
    
    public:

        Astar(Grid& grid, Point& start, Point& goal) {
            this->astarGrid.resize(grid.height, vector<Node>(grid.width));
            findPath(grid, start, goal);
        }

        vector<Point> getPath() {
            return path;
        }
    
};