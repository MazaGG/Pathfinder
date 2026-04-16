#pragma once
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <chrono>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

using namespace std;
using namespace std::chrono;

/*
    Dijkstra Grid Traversal
    - 4-direction movement
    - uniform edge cost = 1
    - grid.cells[y][x] == 1 means blocked
*/

class DijkstraGrid {
private:
    vector<Point> path;
    double elapsedTime;   // milliseconds
    double pathLength;    // number of moves

public:
    DijkstraGrid(
        const Grid& grid,
        const Point& start,
        const Point& goal
    ) {
        solve(grid, start, goal);
    }

    void solve(
        const Grid& grid,
        const Point& start,
        const Point& goal
    ) {
        auto begin = high_resolution_clock::now();

        path.clear();
        elapsedTime = 0.0;
        pathLength = 0.0;

        int width  = (int)grid.width;
        int height = (int)grid.height;

        int sx = (int)start.x;
        int sy = (int)start.y;
        int gx = (int)goal.x;
        int gy = (int)goal.y;

        // invalid start / goal
        if (sx < 0 || sx >= width || sy < 0 || sy >= height ||
            gx < 0 || gx >= width || gy < 0 || gy >= height ||
            grid.cells[sy][sx] == 1 ||
            grid.cells[gy][gx] == 1) {

            auto end = high_resolution_clock::now();
            elapsedTime =
                duration<double, milli>(end - begin).count();
            return;
        }

        vector<vector<AstarCell>> state(
            height,
            vector<AstarCell>(width)
        );

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                state[y][x].isClose = false;
                state[y][x].gScore =
                    numeric_limits<double>::infinity();
                state[y][x].parent = {-1, -1};
            }
        }

        priority_queue<
            ANode,
            vector<ANode>,
            greater<ANode>
        > pq;

        state[sy][sx].gScore = 0.0;
        pq.push({sx, sy, 0.0});

        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        while (!pq.empty()) {
            ANode current = pq.top();
            pq.pop();

            int x = current.x;
            int y = current.y;

            if (state[y][x].isClose)
                continue;

            state[y][x].isClose = true;

            // reached goal
            if (x == gx && y == gy) {
                int cx = gx;
                int cy = gy;

                while (!(cx == -1 && cy == -1)) {
                    path.push_back({
                        (double)cx,
                        (double)cy
                    });

                    pair<int,int> p =
                        state[cy][cx].parent;

                    cx = p.first;
                    cy = p.second;
                }

                reverse(path.begin(), path.end());

                if (path.size() > 1)
                    pathLength =
                        (double)path.size() - 1.0;

                break;
            }

            for (int i = 0; i < 4; i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx < 0 || nx >= width ||
                    ny < 0 || ny >= height)
                    continue;

                if (grid.cells[ny][nx] == 1)
                    continue;

                if (state[ny][nx].isClose)
                    continue;

                double newCost =
                    state[y][x].gScore + 1.0;

                if (newCost <
                    state[ny][nx].gScore) {

                    state[ny][nx].gScore =
                        newCost;

                    state[ny][nx].parent =
                        {x, y};

                    pq.push({
                        nx,
                        ny,
                        newCost
                    });
                }
            }
        }

        auto end = high_resolution_clock::now();

        elapsedTime =
            duration<double, milli>(end - begin).count();
    }

    vector<Point> getPath() const {
        return path;
    }

    double getTime() const {
        return elapsedTime;
    }

    double getLength() const {
        return pathLength;
    }
};