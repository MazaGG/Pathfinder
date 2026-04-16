#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

using namespace std;
using namespace std::chrono;

/*
    BFS Grid Traversal
    - 4-direction movement
    - uniform cost grid (shortest path guaranteed)
*/

class BFSGrid {
private:
    vector<Point> path;
    double elapsedTime;   // milliseconds
    double pathLength;    // number of moves

public:
    BFSGrid(const Grid& grid, const Point& start, const Point& goal) {
        solve(grid, start, goal);
    }

    void solve(const Grid& grid, const Point& start, const Point& goal) {
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

        // invalid checks
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
                state[y][x].gScore = 0.0;
                state[y][x].parent = {-1, -1};
            }
        }

        queue<pair<int,int>> q;

        q.push({sx, sy});
        state[sy][sx].isClose = true;

        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        bool found = false;

        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();

            if (x == gx && y == gy) {
                found = true;
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

                state[ny][nx].isClose = true;
                state[ny][nx].parent = {x, y};

                q.push({nx, ny});
            }
        }

        if (found) {
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