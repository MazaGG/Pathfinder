#pragma once
#include <vector>
#include <queue>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

class FloodFill {

    private:

        Grid reachable;
        vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

        void floodFill(const Grid& grid, const Point& start) {
            int start_x = (int)start.x;
            int start_y = (int)start.y;
            reachable.cells[start_y][start_x];

            queue<pair<int,int>> queue;
            queue.push({start_x,start_y});

            while(!queue.empty()) {
                pair<int,int> current = queue.front();
                queue.pop();

                for (int i = 0; i < directions.size(); i++) {
                    int x = current.first + directions[i].first;
                    int y = current.second + directions[i].second;

                    if (x < 0 || x >= grid.width || y < 0 || y >= grid.height) {
                        continue;
                    }

                    if (grid.cells[y][x] == 1) {
                        continue;
                    }

                    if (reachable.cells[y][x] == 0) {
                        continue;
                    }

                    if (directions[i].first != 0 && directions[i].second != 0) {
                        if (grid.cells[current.second][current.first + directions[i].first] == 1 && grid.cells[current.second + directions[i].second][current.first] == 1) {
                            continue;
                        }
                    }

                    reachable.cells[y][x] = 0;
                    queue.push({x,y});
                }
            }
        }

    public:

        FloodFill(const Grid& grid, const Point& start) {
            this->reachable = Grid{grid.width, grid.height, vector<vector<int>>(grid.height, vector<int>(grid.width, 1))};
            floodFill(grid, start);
        }

        Grid getGrid() {
            return reachable;
        }
};