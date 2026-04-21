#pragma once
#include <vector>
#include <cmath>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

class VoronoiDiagram {

    private:
        Delaunay graph;
        K::Iso_rectangle_2 bbox;

        void buildVoronoiGraph(Grid& grid, vector<Point>& centers) {
            // Insert points
            vector<K::Point_2> points;
            for (int i = 0; i < centers.size(); i++) {
                points.emplace_back(centers[i].x, centers[i].y);
            }
            graph.insert(points.begin(), points.end());
        }

    public:
        VoronoiDiagram(Grid& grid, vector<Point>& centers) {
            K::Iso_rectangle_2 bbox(0.5, 0.5, grid.width - 0.5, grid.height - 0.5);
            this->bbox = bbox;
            buildVoronoiGraph(grid, centers);
        }

        Delaunay getGraph() {
            return graph;
        }

        Delaunay update(Grid& grid, Point& remove, Point& insert) {
            Delaunay::Vertex_handle vh_remove = graph.nearest_vertex(K::Point_2(remove.x, remove.y));
            graph.remove(vh_remove);
            graph.insert(K::Point_2(insert.x, insert.y));
            return graph;
        }

        Delaunay insert(Grid& grid, Point& insert) {
            graph.insert(K::Point_2(insert.x, insert.y));
            return graph;
        }

        Delaunay remove(Grid& grid, Point& remove) {
            Delaunay::Vertex_handle vh_remove = graph.nearest_vertex(K::Point_2(remove.x, remove.y));
            graph.remove(vh_remove);
            return graph;
        }

        void clipRayToGrid (const K::Ray_2& ray, const K::Iso_rectangle_2& bbox, K::Segment_2& clippedSegment) {
            // CGAL::intersection finds the intersection between the ray and the bounding box, which can be a point or a segment.
            CGAL::Object intersection = CGAL::intersection(ray, bbox);

            // typecast to segment
            clippedSegment = CGAL::object_cast<K::Segment_2>(intersection);
        }

};

    