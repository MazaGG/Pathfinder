#pragma once
#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include "struct.hpp"
using namespace std;

double cross(const Point& p, const Point& q, const Point& r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

double distance(const Point& p, const Point& q) {
    return sqrt((p.x - q.x)*(p.x - q.x) + (p.y - q.y)*(p.y - q.y));
}

double manhattan(const int& x1, const int& y1, const int& x2, const int& y2){
    return abs(x1 - x2) + abs (y1 - y2);
}

bool segmentsIntersect(const Point& p1, const Point& p2, const Point& q1, const Point& q2) {
    double d1 = cross(p1, p2, q1);
    double d2 = cross(p1, p2, q2);
    double d3 = cross(q1, q2, p1);
    double d4 = cross(q1, q2, p2);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }
    return false;
}

Point findCenter(const Grid& grid, const Obstacle& obstacle) {
    vector<Point> vertices = obstacle.vertices;
    double x_min = grid.width; 
    double y_min = grid.height;
    double x_max = 0;
    double y_max = 0;

    for(int i = 0; i < vertices.size(); i++) {
        if(vertices[i].x < x_min) {
            x_min = vertices[i].x;
        }
        if(vertices[i].y < y_min) {
            y_min = vertices[i].y;
        }
        if(vertices[i].x > x_max) {
            x_max = vertices[i].x;
        }
        if(vertices[i].y > y_max) {
            y_max = vertices[i].y;
        }
    }
    return { (x_min + x_max) / 2, (y_min + y_max) / 2 };
}

Point intersectionPoint(const Point& p1, const Point& p2, const Point& q1, const Point& q2) {
    double a1 = p2.y - p1.y;
    double b1 = p1.x - p2.x;
    double c1 = a1 * p1.x + b1 * p1.y;
    double a2 = q2.y - q1.y;
    double b2 = q1.x - q2.x;
    double c2 = a2 * q1.x + b2 * q1.y;
    double determinant = a1 * b2 - a2 * b1;

    if (abs(determinant) < 1e-6) {
        return {0, 0};
    }
    return {(b2 * c1 - b1 * c2) / determinant, (a1 * c2 - a2 * c1) / determinant};
}

bool isEdgeValid(const Grid& grid, const Point p, const Point q) { // using Bresenham's line algorithm
    int x0 = (int)p.x;
    int y0 = (int)p.y;
    int x1 = (int)q.x;
    int y1 = (int)q.y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 < 0 || x0 >= grid.width || y0 < 0 || y0 >= grid.height) {
            return false;
        }
        if (grid.cells[y0][x0] == 1) {
            return false;
        }
        if (x0 == x1 && y0 == y1) {
            break;
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
    return true;
}

// Refactor: should we early exit, nearest, or should we find best ?
// note: current implementation is actually not nearest, but best
int findNearestVisibleVertex (const Point current, const Point goal, const vector<VoronoiVertex>& vertices, const vector<bool>& visited, const Grid& grid) {
    int n = vertices.size();

    // small graph, no sorting
    if (n < 100) {
        int bestIdx = -1;
        double bestCost = numeric_limits<double>::infinity();

        for (int i = 0; i < n; i++) {
            Point next = vertices[i].position;
            double cost = distance(current, next);

            if (visited[i]) {
                continue;
            }
            if (distance(current, goal) < distance(next, goal)) {
                continue;
            } 
            if (!isEdgeValid(grid, current, next)) {
                continue;
            }
            if (cost < bestCost) {
                bestCost = cost;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    // large graph, sort indices
    vector<pair<double,int>> order;
    order.reserve(n);

    for (int i = 0; i < n; i++) {
        double cost = distance(current, vertices[i].position);
        order.push_back({cost, i});
    }

    sort(order.begin(), order.end());
    for (int i = 0; i < n; i++) {
        int nextIdx = order[i].second;
        if (isEdgeValid(grid, current, vertices[nextIdx].position) && !visited[nextIdx]) {
            return nextIdx; 
        }
    }

    return -1;
}

double computePathLength(const vector<Point>& path) {
    if (path.size() < 2) return 0.0;

    double len = 0.0;

    for (int i = 1; i < path.size(); i++) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        len += sqrt(dx * dx + dy * dy);
    }

    return len;
}

Point snapToGridBoundary(const Point& p, Grid& grid) {
    const double EPS = 1e-6;
    const double OFFSET = 0.5;
    Point result = p;
    
    // Left boundary (x = 0) -> set to 0.5
    if (abs(p.x) < EPS) {
        result.x = OFFSET;
    }
    // Right boundary (x = grid.width) -> set to grid.width - 0.5
    else if (abs(p.x - grid.width) < EPS) {
        result.x = grid.width - OFFSET;
    }
    
    // Bottom boundary (y = 0) -> set to 0.5
    if (abs(p.y) < EPS) {
        result.y = OFFSET;
    }
    // Top boundary (y = grid.height) -> set to grid.height - 0.5
    else if (abs(p.y - grid.height) < EPS) {
        result.y = grid.height - OFFSET;
    }
    
    return result;
}