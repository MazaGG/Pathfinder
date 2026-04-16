#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <algorithm>
#include <chrono>

#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"

using namespace std;

struct RRTNode {
    Point p;
    int parent;
};

class RRT {
public:
    // -------------------------
    // PUBLIC INTERFACE
    // -------------------------
    RRT(const Grid& grid, Point start, Point goal,
        double stepSize = 5.0,
        int maxIterations = 10000)
        : grid(grid),
          start(start),
          goal(goal),
          stepSize(stepSize),
          maxIterations(maxIterations)
    {
        tree.clear();
        tree.push_back({start, -1});

        solved = false;
        goalIndex = -1;
        buildTime = 0.0;
        run();
    }

    // Run RRT
    void run() {
        auto t1 = chrono::high_resolution_clock::now();

        for (int i = 0; i < maxIterations; i++) {

            Point randPoint = sample();
            int nearestIdx = getNearest(randPoint);

            Point newPoint = steer(tree[nearestIdx].p, randPoint);

            // YOUR collision function (critical)
            if (!isEdgeValidRRT(grid, tree[nearestIdx].p, newPoint)) {
                continue;
            }

            tree.push_back({newPoint, nearestIdx});
            int newIdx = (int)tree.size() - 1;

            // goal check
            if (distance(newPoint, goal) < stepSize * 2.0 &&
                isEdgeValidRRT(grid, newPoint, goal))
            {
                tree.push_back({goal, newIdx});
                goalIndex = (int)tree.size() - 1;
                solved = true;
                break;
            }
        }

        auto t2 = chrono::high_resolution_clock::now();
        buildTime = chrono::duration<double>(t2 - t1).count();
    }

    // -------------------------
    // GET PATH
    // -------------------------
    vector<Point> getPath() {
        vector<Point> path;

        if (!solved) return path;

        int cur = goalIndex;

        while (cur != -1) {
            path.push_back(tree[cur].p);
            cur = tree[cur].parent;
        }

        reverse(path.begin(), path.end());
        return path;
    }

    // -------------------------
    // GET TIME
    // -------------------------
    double getTime() const {
        return buildTime;
    }

    bool isSolved() const {
        return solved;
    }

private:
    // -------------------------
    // DATA
    // -------------------------
    const Grid& grid;

    vector<RRTNode> tree;

    Point start, goal;

    double stepSize;
    int maxIterations;

    bool solved;
    int goalIndex;

    double buildTime;

    // -------------------------
    // CORE FUNCTIONS
    // -------------------------

    Point sample() {
        return {
            (double)(rand() % (int)grid.width),
            (double)(rand() % (int)grid.height)
        };
    }

    int getNearest(const Point& q) {
        int best = 0;
        double bestDist = numeric_limits<double>::infinity();

        for (int i = 0; i < tree.size(); i++) {
            double d = distance(tree[i].p, q);
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }

        return best;
    }

    Point steer(const Point& from, const Point& to) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;

        double d = sqrt(dx * dx + dy * dy);
        if (d < stepSize) return to;

        return {
            from.x + (dx / d) * stepSize,
            from.y + (dy / d) * stepSize
        };
    }

    double distance(const Point& a, const Point& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return sqrt(dx * dx + dy * dy);
    }

    bool isEdgeValidRRT(const Grid& grid, const Point& a, const Point& b) {
        const double step = 0.25;  // small enough to not skip obstacles

        double dx = b.x - a.x;
        double dy = b.y - a.y;

        double dist = sqrt(dx * dx + dy * dy);
        if (dist == 0) return true;

        int steps = (int)(dist / step);

        double xInc = dx / steps;
        double yInc = dy / steps;

        double x = a.x;
        double y = a.y;

        for (int i = 0; i <= steps; i++) {
            int gx = (int)floor(x);
            int gy = (int)floor(y);

            // out of bounds = collision
            if (gx < 0 || gy < 0 || gx >= (int)grid.width || gy >= (int)grid.height)
                return false;

            // occupied cell = collision
            if (grid.cells[gy][gx] == 1)
                return false;

            x += xInc;
            y += yInc;
        }

        return true;
    }
};