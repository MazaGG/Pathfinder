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
        double margin = 1e-9;
        Grid grid;
        vector<Obstacle> obstacles;

        bool isOutsideGrid(Point& p, Grid& grid) {
            return p.x < 0 - margin || p.x > grid.width + margin || p.y < 0 - margin || p.y > grid.height + margin;
        }

        Point gridIntersection(Point& p1, Point& p2, Grid& grid) {
            // left border
            if ((p1.x < 0 && p2.x >= 0) || (p1.x >= 0 && p2.x < 0)) {
                return intersectionPoint(p1, p2, {0, 0}, {0, grid.height});
            }
            // right border
            else if ((p1.x < grid.width && p2.x >= grid.width) || (p1.x >= grid.width && p2.x < grid.width)) {
                return intersectionPoint(p1, p2, {grid.width, 0}, {grid.width, grid.height});
            }
            // top border
            else if ((p1.y < 0 && p2.y >= 0) || (p1.y >= 0 && p2.y < 0)) {
                return intersectionPoint(p1, p2, {0, 0}, {grid.width, 0});
            }
            // top border
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

        void createObstacle(Obstacle& obstacle) {
            int numVertices = rand() % v_max + 3;
            vector<Point> vertices;
            vector<double> angles;
            Point center = {(double)(rand() % (int)grid.width), (double)(rand() % (int)grid.height)};

            // generate random angles
            double PI = 3.14159265358979323846;
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
            obstacle = {clip(vertices, grid), center};

            // generate random velocity
            double speed = 0.5 + ((double)rand() / RAND_MAX) * 4.5;  // 0.5 to 2.5 pixels/frame
            double direction = ((double)rand() / RAND_MAX) * 2 * PI;  // Random angle
            obstacle.velocity_x = speed * cos(direction);
            obstacle.velocity_y = speed * sin(direction);
        }

        void move() {
            for (int i = 0; i < obstacles.size(); i++) {
                Obstacle& obstacle = obstacles[i];

                // 10% chance to change velocity
                if (rand() % 10 == 0) {
                    double PI = 3.14159265358979323846;
                    double speed = 0.5 + ((double)rand() / RAND_MAX) * 4.5;  // 0.5 to 2.5 pixels/frame
                    double direction = ((double)rand() / RAND_MAX) * 2 * PI;  // Random angle
                    obstacle.velocity_x = speed * cos(direction);
                    obstacle.velocity_y = speed * sin(direction);
                }

                for (int j = 0; j < obstacle.vertices.size(); j++) {
                    Point& vertex = obstacle.vertices[j];
                    vertex.x += obstacle.velocity_x;
                    vertex.y += obstacle.velocity_y;
                }
                obstacle.center.x += obstacle.velocity_x;
                obstacle.center.y += obstacle.velocity_y;
                obstacle.vertices = clip(obstacle.vertices, grid);
                
                // If obstacle is completely clipped away (empty vertices), respawn it
                if (obstacle.vertices.empty()) {
                    createObstacle(obstacle);
                }
            }
        }


    public:

        ObstacleGenerator(int seed, int n, int v_max=10, int radius_min=20, int radius_max=40, Grid grid={100, 100}) {
            srand(seed);
            this->n = n;
            this->v_max = v_max;
            this->radius_min = radius_min;
            this->radius_max = radius_max;
            this->grid = grid;
        }

        vector<Obstacle> generateObstacles() {

            while (obstacles.size() < n) {
                Obstacle obstacle;
                createObstacle(obstacle);
                obstacles.push_back(obstacle);
            }

            return obstacles;
        }
        
        vector<Obstacle> moveObstacles() {
            move();
            return obstacles;
        }
};