#pragma once
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <vector>
#include <cmath>
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 CGALPoint;

class VoronoiDiagram {

    private:
        Grid grid;
        vector<Obstacle> obstacles;
        vector<VoronoiVertex> vertices;

        vector<VoronoiVertex> buildVoronoiGraph(vector<Obstacle>& obstacles) {
            // insert points
            vector<CGALPoint> points;
            for (int i = 0; i < obstacles.size(); i++) {
                points.emplace_back(obstacles[i].center.x, obstacles[i].center.y);
            }

            Delaunay dt;
            dt.insert(points.begin(), points.end());
            map<Delaunay::Face_handle, int> faceToIndex;
            vector<VoronoiVertex> vertices;
            int idx = 0;

            // create voronoi vertices (circumcenters)
            for (Delaunay::Finite_faces_iterator f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
                CGALPoint cc = dt.dual(f);
                VoronoiVertex v;
                v.position = {cc.x(), cc.y()};
                vertices.push_back(v);
                faceToIndex[f] = idx++;
            }

            // connect neighbors
            for (Delaunay::Finite_faces_iterator f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
                int v1 = faceToIndex[f];

                for (int i = 0; i < 3; i++) {
                    Delaunay::Face_handle neighbor = f->neighbor(i);
                    if (!dt.is_infinite(neighbor)) {
                        int v2 = faceToIndex[neighbor];
                        vertices[v1].neighbors.push_back(v2);
                    }
                }
            }

            return vertices;
        }

    public:
    
        VoronoiDiagram(vector<Obstacle>& obstacles, Grid& grid) {
            this->obstacles = obstacles;
            this->grid = grid;
        }

        vector<VoronoiVertex> getVertices() {
            for (int i = 0; i < obstacles.size(); i++) {
                obstacles[i].center = findCenter(grid, obstacles[i]);
            }

            vertices = buildVoronoiGraph(obstacles);
            return vertices;
        }

};

    