#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <limits>

#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

using namespace std;

class JPSGrid {
private:
    Grid grid;
    Point start, goal;

    vector<Point> path;
    double runtime = 0.0;

    struct Node {
        int x, y;
        double g, f;

        bool operator>(const Node& other) const {
            return f > other.f;
        }
    };

    bool inBounds(int x, int y) {
        return x >= 0 && y >= 0 &&
               x < (int)grid.width &&
               y < (int)grid.height;
    }

    bool isBlocked(int x, int y) {
        return grid.cells[y][x] == 1;
    }

    double heuristic(int x, int y) {
        return manhattan(x, y, (int)goal.x, (int)goal.y);
    }

    bool isGoal(int x, int y) {
        return x == (int)goal.x && y == (int)goal.y;
    }

    // move direction pruning (JPS core idea simplified for 4-dir)
    pair<int,int> pruneDirection(int px, int py, int dx, int dy) {
        return {dx, dy};
    }

    // recursive jump function
    pair<int,int> jump(int x, int y, int dx, int dy) {
        int nx = x + dx;
        int ny = y + dy;

        if (!inBounds(nx, ny) || isBlocked(nx, ny))
            return {-1, -1};

        if (isGoal(nx, ny))
            return {nx, ny};

        // forced neighbor check (4-dir simplified)
        if (dx != 0) {
            if ((inBounds(nx, ny + 1) && isBlocked(nx, ny + 1) &&
                 inBounds(x, ny + 1) && !isBlocked(x, ny + 1)) ||
                (inBounds(nx, ny - 1) && isBlocked(nx, ny - 1) &&
                 inBounds(x, ny - 1) && !isBlocked(x, ny - 1))) {
                return {nx, ny};
            }
        }

        if (dy != 0) {
            if ((inBounds(nx + 1, ny) && isBlocked(nx + 1, ny) &&
                 inBounds(nx + 1, y) && !isBlocked(nx + 1, y)) ||
                (inBounds(nx - 1, ny) && isBlocked(nx - 1, ny) &&
                 inBounds(nx - 1, y) && !isBlocked(nx - 1, y))) {
                return {nx, ny};
            }
        }

        return jump(nx, ny, dx, dy);
    }

    vector<pair<int,int>> getNeighbors(int x, int y) {
        return {
            {1,0}, {-1,0}, {0,1}, {0,-1}
        };
    }

public:
    JPSGrid(const Grid& g, const Point& s, const Point& t)
        : grid(g), start(s), goal(t) {
            findPath();
        }

    void findPath() {
        auto startTime = chrono::high_resolution_clock::now();

        priority_queue<Node, vector<Node>, greater<Node>> open;
        unordered_map<int, double> gScore;
        unordered_map<int, pair<int,int>> parent;
        unordered_map<int, bool> closed;

        auto hash = [&](int x, int y) {
            return y * (int)grid.width + x;
        };

        int sx = (int)start.x, sy = (int)start.y;
        int gx = (int)goal.x,  gy = (int)goal.y;

        open.push({sx, sy, 0, heuristic(sx, sy)});
        gScore[hash(sx, sy)] = 0;

        bool found = false;

        while (!open.empty()) {
            Node cur = open.top();
            open.pop();

            int cx = cur.x, cy = cur.y;
            int ckey = hash(cx, cy);

            if (closed[ckey]) continue;
            closed[ckey] = true;

            if (isGoal(cx, cy)) {
                found = true;
                break;
            }

            for (auto [dx, dy] : getNeighbors(cx, cy)) {
                auto jp = jump(cx, cy, dx, dy);

                if (jp.first == -1) continue;

                int jx = jp.first;
                int jy = jp.second;
                int jkey = hash(jx, jy);

                double newG = gScore[ckey] + manhattan(cx,cy,jx,jy);

                if (!gScore.count(jkey) || newG < gScore[jkey]) {
                    gScore[jkey] = newG;
                    parent[jkey] = {cx, cy};

                    open.push({
                        jx,
                        jy,
                        newG,
                        newG + heuristic(jx, jy)
                    });
                }
            }
        }

        // reconstruct
        path.clear();

        if (found) {
            int cx = gx, cy = gy;

            while (!(cx == sx && cy == sy)) {
                path.push_back({(double)cx, (double)cy});
                int key = hash(cx, cy);

                auto p = parent[key];
                cx = p.first;
                cy = p.second;
            }

            path.push_back(start);
            reverse(path.begin(), path.end());
        }

        auto endTime = chrono::high_resolution_clock::now();
        runtime = chrono::duration<double, milli>(endTime - startTime).count();
    }

    vector<Point> getPath() {
        return path;
    }

    double getTime() {
        return runtime;
    }
};