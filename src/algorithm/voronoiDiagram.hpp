#pragma once
#include <vector>
#include <cmath>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, CGAL::Default, Itag> CDT;

class VoronoiDiagram {

    private:
        CDT graph;
        vector<VoronoiVertex> vertices;
        K::Iso_rectangle_2 bbox;

        void buildVoronoiGraph(Grid& grid, vector<Point>& centers) {
            // insert points
            vector<CDT::Point> points;
            for (int i = 0; i < centers.size(); i++) {
                points.emplace_back(centers[i].x, centers[i].y);
            }
            graph.insert(points.begin(), points.end());

            // insert border constraints
            graph.insert_constraint(K::Point_2(0.5, 0.5), K::Point_2(0.5, grid.height - 0.5));
            graph.insert_constraint(K::Point_2(0.5, 0.5), K::Point_2(grid.width - 0.5, 0.5));
            graph.insert_constraint(K::Point_2(grid.width - 0.5, 0.5), K::Point_2(grid.width - 0.5, grid.height - 0.5));
            graph.insert_constraint(K::Point_2(0.5, grid.height - 0.5), K::Point_2(grid.width - 0.5, grid.height - 0.5));

            map<CDT::Face_handle, int> faceToIndex;
            int idx = 0;

            // find voronoi vertices
            for (CDT::Finite_faces_iterator i = graph.finite_faces_begin(); i != graph.finite_faces_end(); i++) {
                CDT::Face_handle face = i;
                K::Point_2 circumcenter = graph.circumcenter(face);
                VoronoiVertex vertex;
                vertex.position = {circumcenter.x(), circumcenter.y()};
                vertices.push_back(vertex);
                faceToIndex[face] = idx;
                idx++;
            }

            // connect voronoi vertices
            for (CDT::Finite_faces_iterator i = graph.finite_faces_begin(); i != graph.finite_faces_end(); i++) {
                int vertex1 = faceToIndex[i];
                for (int j = 0; j < 3; j++) {
                    CDT::Face_handle neighborFace = i->neighbor(j);
                    if (!graph.is_infinite(neighborFace)) {
                        int vertex2 = faceToIndex[neighborFace];
                        vertices[vertex1].neighbors.push_back(vertex2);
                    }
                }
            }
        }

    public:
        VoronoiDiagram(Grid& grid, vector<Point>& centers) {
            K::Iso_rectangle_2 bbox(0.5, 0.5, grid.width - 0.5, grid.height - 0.5);
            this->bbox = bbox;
            buildVoronoiGraph(grid, centers);
        }

        CDT getGraph() {
            return graph;
        }

        vector<VoronoiVertex> getVertices() {
            return vertices;
        }

        CDT update(Grid& grid, CDT::Vertex_handle remove, Point& insert) {
            graph.remove(remove);
            graph.insert(K::Point_2(insert.x, insert.y));
            return graph;
        }

        CDT insert(Grid& grid, Point& insert) {
            graph.insert(K::Point_2(insert.x, insert.y));
            return graph;
        }

        CDT remove(Grid& grid, CDT::Vertex_handle vh_remove) {
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

    