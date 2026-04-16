#pragma once
#include <vector>
#include <limits>
using namespace std;

struct Point {
    double x, y;
};

struct Grid {
    double width, height;
    std::vector<std::vector<int>> cells;
};

struct Obstacle {
    std::vector<Point> vertices;
    Point center;
};

struct VoronoiVertex {
    Point position;
    std::vector<int> neighbors;
};

struct ANode {
    int x, y;
    double score;
    bool operator>(const ANode& other) const {
        return score > other.score;
    }
};

struct VNode {
    int idx;
    double score;
    bool operator>(const VNode& other) const {
        return score > other.score;
    }
};

struct AstarCell {
    bool isClose;
    double gScore;
    pair<int,int> parent;
};