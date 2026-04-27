#pragma once
#include <vector>
#include <limits>
using namespace std;

struct Point {
    double x, y;

    bool operator == (const Point& other) const {
        return (x == other.x && y == other.y);
    }

    bool operator != (const Point& other) const {
        return (x != other.x || y != other.y);
    }
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
    int index;
    int parentIndex = -1;
    bool isClosed = false;
    double gscore = numeric_limits<double>::infinity();
    double score = numeric_limits<double>::infinity();

    bool operator > (const VoronoiVertex& other) const {
        return score > other.score;
    }

    bool operator == (const VoronoiVertex& other) const {
        return (position.x == other.position.x && position.y == other.position.y);
    }
};

struct AstarNode {
    int x, y;
    int parentX = -1;
    int parentY = -1;
    int closedId = -1;
    int searchId = -1;
    int gscoreId = -1;
    double gscore = numeric_limits<double>::infinity();
    double score = numeric_limits<double>::infinity();

    bool operator > (const AstarNode& other) const {
        return score > other.score;
    }
};

struct DJKNode {
    int x, y;
    int parentX = -1;
    int parentY = -1;
    bool isClosed = false;
    double gscore = numeric_limits<double>::infinity();

    bool operator > (const DJKNode& other) const {
        return gscore > other.gscore;
    }
};