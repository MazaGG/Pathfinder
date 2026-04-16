#pragma once
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include <algorithm>
#include <cmath>

using namespace std;

class OccupancyMapGenerator {

    private:
        Grid grid;

        bool pointInPolygon(const Point p, const vector<Point>& vertices) {
            int n = vertices.size();
            bool inside = false;

            for (int i = 0, j = n - 1; i < n; j = i++) {
                double xi = vertices[i].x, yi = vertices[i].y;
                double xj = vertices[j].x, yj = vertices[j].y;

                bool intersect = ((yi > p.y) != (yj > p.y)) &&
                                (p.x < (xj - xi) * (p.y - yi) / (yj - yi + 1e-9) + xi);

                if (intersect)
                    inside = !inside;
            }

            return inside;
        }

        void drawLine(Grid& grid, const Point p, const Point q) {
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
                if (x0 >= 0 && x0 < grid.width && y0 >= 0 && y0 < grid.height) {
                    grid.cells[y0][x0] = 1;
                }
                if (x0 == x1 && y0 == y1) break;

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
        }

    public:

        OccupancyMapGenerator(Grid grid) {
            this->grid = grid;
        }

        Grid generateOccupancyMap(vector<Obstacle>& obstacles, Grid& grid) {

            for (int y = 0; y < grid.height; y++) {
                for (int x = 0; x < grid.width; x++) {
                    Point p = {x + 0.5, y + 0.5};

                    for (int i = 0; i < obstacles.size(); i++) {
                        if (pointInPolygon(p, obstacles[i].vertices)) {
                            grid.cells[y][x] = 1;
                        }
                    }
                }
            }

            for (int i = 0; i < obstacles.size(); i++) {
                for (int j = 0; j < obstacles[i].vertices.size(); j++) {
                    Point p = obstacles[i].vertices[j];
                    Point q = obstacles[i].vertices[(j + 1) % obstacles[i].vertices.size()];
                    drawLine(grid, p, q);
                }
            }

            return grid;
        }
};