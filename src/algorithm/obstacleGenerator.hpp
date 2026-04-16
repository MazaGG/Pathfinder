#pragma once
#include <cmath>
#include <algorithm>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
using namespace std;

class ObstacleGenerator {

    private:
        int n;
        int v_max;
        int radius_min;
        int radius_max;
        Grid grid;
        vector<Obstacle> obstacles;

        bool isOutsideGrid(Point& p, Grid& grid) {
            double margin = 1e-9;
            return p.x < margin || p.x > grid.width - margin || p.y < margin || p.y > grid.height - margin;
        }

        Point gridIntersection(Point& p1, Point& p2, Grid& grid) {
            if ((p1.x < 0 && p2.x >= 0) || (p1.x >= 0 && p2.x < 0)) {
                return intersectionPoint(p1, p2, {0, 0}, {0, grid.height});
            }
            else if ((p1.x < grid.width && p2.x >= grid.width) || (p1.x >= grid.width && p2.x < grid.width)) {
                return intersectionPoint(p1, p2, {grid.width, 0}, {grid.width, grid.height});
            }
            else if ((p1.y < 0 && p2.y >= 0) || (p1.y >= 0 && p2.y < 0)) {
                return intersectionPoint(p1, p2, {0, 0}, {grid.width, 0});
            }
            else {
                return intersectionPoint(p1, p2, {0, grid.height}, {grid.width, grid.height});
            }
        }

        vector<Point> clip(vector<Point>& vertices, Grid& grid) { // using Sutherland-Hodgman algorithm for polygon clipping
            vector<Point> clippedVertices;
            for (int i = 0; i < vertices.size(); i++) {

                int prev = i - 1;
                if (prev < 0) {
                    prev += vertices.size();
                }
                
                Point v1 = vertices[prev];
                Point v2 = vertices[i];

                // in-out
                if (!isOutsideGrid(v1, grid) && isOutsideGrid(v2, grid)) {
                    Point intersection = gridIntersection(v1, v2, grid);
                    clippedVertices.push_back(intersection);
                }
                // out-in
                else if (isOutsideGrid(v1, grid) && !isOutsideGrid(v2, grid)) {
                    Point intersection = gridIntersection(v1, v2, grid);
                    clippedVertices.push_back(intersection);
                    clippedVertices.push_back(v2);
                }
                // in-in
                else if (!isOutsideGrid(v1, grid) && !isOutsideGrid(v2, grid)) {
                    clippedVertices.push_back(v2);
                }
            }
            return clippedVertices;
        }


    public:

        ObstacleGenerator(int seed, int n, int v_max, int radius_min=5, int radius_max=20, Grid grid={100, 100}) {
            srand(seed);
            this->n = n;
            this->v_max = v_max;
            this->radius_min = radius_min;
            this->radius_max = radius_max;
            this->grid = grid;
        }

        vector<Obstacle> generateObstacles() {

            while (obstacles.size() < n) {
                int numVertices = rand() % v_max + 3;
                vector<Point> vertices;
                vector<double> angles;
                Point center = {(double)(rand() % (int)grid.width), (double)(rand() % (int)grid.height)};

                // generate random angles
                constexpr double PI = 3.14159265358979323846;
                for (int i = 0; i < numVertices; i++) {
                    double angle = ((double)rand() / RAND_MAX) * 2 * PI;
                    angles.push_back(angle);
                }
                sort(angles.begin(), angles.end());
                // generate vertices
                for (int i = 0; i < numVertices; i++) {
                    double radius = radius_min + ((double)rand() / RAND_MAX) * (radius_max - radius_min);
                    Point p;
                    p.x = center.x + radius * cos(angles[i]);
                    p.y = center.y + radius * sin(angles[i]);
                    vertices.push_back(p);
                }
                // clip obstacles
                Obstacle obstacle = {clip(vertices, grid), {0, 0}};
                obstacles.push_back(obstacle);
            }

            return obstacles;
        }
        
};