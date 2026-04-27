#pragma once
#include <vector>
#include <cmath>
#include <variant>
#include <chrono>
#include <map>
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
        vector<VoronoiVertex> vertices;
        K::Iso_rectangle_2 bbox;

        bool isPointInGrid(const K::Point_2& point) {
            if (point.x() <= 0.5 || point.x() >= bbox.xmax() || point.y() <= 0.5 || point.y() >= bbox.ymax()) {
                return false;
            }
            return true;
        }

        void buildVoronoiGraph(Grid& grid, vector<Point>& centers) {
            auto voronoi_start = chrono::high_resolution_clock::now();

            // insert points
            vector<K::Point_2> points;
            for (int i = 0; i < centers.size(); i++) {
                points.push_back(K::Point_2(centers[i].x, centers[i].y));
            }
            graph.insert(points.begin(), points.end());
        }

        void buildVoronoiVertices(Grid& grid) {
            // maps delaunay triangle to indices for neighbor lookup in voronoi vertices
            map<Delaunay::Face_handle, int> faceIndexMap; 
            int faceIndex = 0;

            for (Delaunay::Finite_faces_iterator i = graph.finite_faces_begin(); i != graph.finite_faces_end(); ++i) {
                K::Point_2 circumcenter = graph.circumcenter(i);
                if (isPointInGrid(circumcenter)) { // only consider circumcenters that are within the bounding box
                    VoronoiVertex vertex;
                    vertex.position = Point{circumcenter.x(), circumcenter.y()};
                    vertex.index = vertices.size();
                    vertices.push_back(vertex);
                    faceIndexMap[i] = faceIndex++;
                } else {
                    faceIndexMap[i] = -1; // mark as invalid
                }
            }

            vector<pair<int, double>> leftBoundaryVertices;
            vector<pair<int, double>> rightBoundaryVertices;
            vector<pair<int, double>> topBoundaryVertices;
            vector<pair<int, double>> bottomBoundaryVertices;

            // build neighbor relationships
            for (Delaunay::Finite_faces_iterator i = graph.finite_faces_begin(); i != graph.finite_faces_end(); ++i) {
                int currentIndex = faceIndexMap[i];
                if (currentIndex != -1) {
                    for (int j = 0; j < 3; j++) {
                        Delaunay::Face_handle neighborFace = i->neighbor(j);

                        if (!graph.is_infinite(neighborFace)) {
                            int neighborIndex = faceIndexMap[neighborFace];
                            if (neighborIndex != -1) {
                                vertices[currentIndex].neighbors.push_back(neighborIndex); 
                            }
                            // the neighbor face is not infinite, but its circumcenter is outside bbox
                            // thus, we find the segment up to the bbox, and mark it as boundary
                            // Refactor: can turn this into a function since iz used twice
                            else {
                                Delaunay::Edge edge(i, j);
                                CGAL::Object dual = graph.dual(edge);
                                const K::Segment_2* voronoi_edge = CGAL::object_cast<K::Segment_2>(&dual);
                                CGAL::Object intersection = CGAL::intersection(bbox, *voronoi_edge);
                                const K::Segment_2* segment = CGAL::object_cast<K::Segment_2>(&intersection);

                                if (segment) {
                                    findSegmentInBbox(segment, currentIndex, leftBoundaryVertices, topBoundaryVertices, rightBoundaryVertices, bottomBoundaryVertices);
                                }
                            }
                        } 
                        else {
                            // get the perpendicular ray and find the intersection with the bounding box
                            // add to the list of neighbors for the current vertex
                            // add to the boundary vertices vector<int> boundaryVertices;
                            Delaunay::Edge edge(i, j);
                            CGAL::Object dual = graph.dual(edge);
                            const K::Ray_2* ray = CGAL::object_cast<K::Ray_2>(&dual);

                            if (ray) {
                                CGAL::Object intersection = CGAL::intersection(bbox, *ray);
                                const K::Segment_2* segment = CGAL::object_cast<K::Segment_2>(&intersection);

                                if (segment) {
                                    findSegmentInBbox(segment, currentIndex, leftBoundaryVertices, topBoundaryVertices, rightBoundaryVertices, bottomBoundaryVertices);
                                }
                            }
                        }
                    }
                }
            }

            connectBorderVertices(grid, leftBoundaryVertices, topBoundaryVertices, rightBoundaryVertices, bottomBoundaryVertices);
        }

        void findSegmentInBbox(const K::Segment_2* segment, int& currentIndex, vector<pair<int, double>>& leftBoundaryVertices, vector<pair<int, double>>& topBoundaryVertices, vector<pair<int, double>>& rightBoundaryVertices, vector<pair<int, double>>& bottomBoundaryVertices) {
            VoronoiVertex boundaryVertex;
            boundaryVertex.position = Point{segment->target().x(), segment->target().y()};
            boundaryVertex.index = vertices.size();
            boundaryVertex.neighbors.push_back(currentIndex);
            
            vertices.push_back(boundaryVertex);
            vertices[currentIndex].neighbors.push_back(vertices.size() - 1);
            
            double margin = 1e-6;
            double x = segment->target().x();
            double y = segment->target().y();
            
            // Use epsilon comparisons!
            if (abs(x - 0.5) < margin) {
                leftBoundaryVertices.push_back({vertices.size() - 1, y});
            } else if (abs(x - bbox.xmax()) < margin) {
                rightBoundaryVertices.push_back({vertices.size() - 1, bbox.ymax() - y});
            } else if (abs(y - 0.5) < margin) {
                bottomBoundaryVertices.push_back({vertices.size() - 1, bbox.xmax() - x});
            } else if (abs(y - bbox.ymax()) < margin) {
                topBoundaryVertices.push_back({vertices.size() - 1, x});
            }
            // If none match, the vertex isn't on a boundary — it's still added to vertices
            // but not to any boundary list
        }

        void connectBorderVertices(Grid& grid, vector<pair<int, double>>& leftBoundaryVertices, vector<pair<int, double>>& topBoundaryVertices, vector<pair<int, double>>& rightBoundaryVertices, vector<pair<int, double>>& bottomBoundaryVertices) {
            sort(leftBoundaryVertices.begin(), leftBoundaryVertices.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
                return a.second < b.second; // sort by order along the boundary
            });
            sort(topBoundaryVertices.begin(), topBoundaryVertices.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
                return a.second < b.second; // sort by order along the boundary
            });
            sort(rightBoundaryVertices.begin(), rightBoundaryVertices.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
                return a.second < b.second; // sort by order along the boundary
            });
            sort(bottomBoundaryVertices.begin(), bottomBoundaryVertices.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
                return a.second < b.second;
            });

            // Refactor: please save your future self, cuz why did I choose long-ass names
            // left -> corner -> top -> corner -> right -> corner -> bottom -> corner
            // left (includes bottom left and top left corner, if any)
            if (leftBoundaryVertices.size() > 0) {
                for (int i = 0; i < leftBoundaryVertices.size() - 1; i++) {
                    if (isEdgeValid(grid, vertices[leftBoundaryVertices[i].first].position, vertices[leftBoundaryVertices[i + 1].first].position)) {
                        vertices[leftBoundaryVertices[i].first].neighbors.push_back(leftBoundaryVertices[i + 1].first);
                        vertices[leftBoundaryVertices[i + 1].first].neighbors.push_back(leftBoundaryVertices[i].first);
                    }
                }
            }

            // top left corner
            if (grid.cells[grid.height - 1][0] == 0) {
                if (vertices[leftBoundaryVertices.back().first].position.y != grid.height - 0.5) {
                    VoronoiVertex corner;
                    corner.position = Point{0.5, grid.height - 0.5};
                    corner.index = vertices.size();
                    vertices.push_back(corner);
                    if (isEdgeValid(grid, vertices[leftBoundaryVertices.back().first].position, corner.position)) {
                        vertices[leftBoundaryVertices.back().first].neighbors.push_back(vertices.size() - 1);
                        vertices.back().neighbors.push_back(leftBoundaryVertices.back().first);
                    }
                    leftBoundaryVertices.push_back(pair(vertices.size() - 1, grid.height - 0.5));
                }
                if (isEdgeValid(grid, vertices[leftBoundaryVertices.back().first].position, vertices[topBoundaryVertices[0].first].position)) {
                    vertices[leftBoundaryVertices.back().first].neighbors.push_back(topBoundaryVertices[0].first);
                    vertices[topBoundaryVertices[0].first].neighbors.push_back(leftBoundaryVertices.back().first);
                }
            }

            // top
            if (topBoundaryVertices.size() > 0) {
                for (int i = 0; i < topBoundaryVertices.size() - 1; i++) {
                    if (isEdgeValid(grid, vertices[topBoundaryVertices[i].first].position, vertices[topBoundaryVertices[i + 1].first].position)) {
                        vertices[topBoundaryVertices[i].first].neighbors.push_back(topBoundaryVertices[i + 1].first);
                        vertices[topBoundaryVertices[i + 1].first].neighbors.push_back(topBoundaryVertices[i].first);
                    }
                }
            }

            // top right corner
            if (grid.cells[grid.height - 1][grid.width - 1] == 0) {
                if (vertices[rightBoundaryVertices[0].first].position.y != grid.height - 0.5) {
                    VoronoiVertex corner;
                    corner.position = Point{grid.width - 0.5, grid.height - 0.5};
                    corner.index = vertices.size();
                    vertices.push_back(corner);
                    rightBoundaryVertices.insert(rightBoundaryVertices.begin(), pair(vertices.size() - 1, 0.5));
                }
                if (isEdgeValid(grid, vertices[topBoundaryVertices.back().first].position, vertices[rightBoundaryVertices[0].first].position)) {
                    vertices[topBoundaryVertices.back().first].neighbors.push_back(rightBoundaryVertices[0].first);
                    vertices[rightBoundaryVertices[0].first].neighbors.push_back(topBoundaryVertices.back().first);
                }
            }

            // right (includes bottom right and top right corner, if any)
            if (rightBoundaryVertices.size() > 0) {
                for (int i = 0; i < rightBoundaryVertices.size() - 1; i++) {
                    if (isEdgeValid(grid, vertices[rightBoundaryVertices[i].first].position, vertices[rightBoundaryVertices[i + 1].first].position)) {
                        vertices[rightBoundaryVertices[i].first].neighbors.push_back(rightBoundaryVertices[i + 1].first);
                        vertices[rightBoundaryVertices[i + 1].first].neighbors.push_back(rightBoundaryVertices[i].first);
                    }
                }
            }

            // bottom right corner
            if (grid.cells[0][grid.width - 1] == 0) {
                if (vertices[rightBoundaryVertices.back().first].position.y != 0.5) {
                    VoronoiVertex corner;
                    corner.position = Point{grid.width - 0.5, 0.5};
                    corner.index = vertices.size();
                    vertices.push_back(corner);
                    if (isEdgeValid(grid, vertices[rightBoundaryVertices.back().first].position, corner.position)) {
                        vertices[rightBoundaryVertices.back().first].neighbors.push_back(vertices.size() - 1);
                        vertices.back().neighbors.push_back(rightBoundaryVertices.back().first);
                    }
                    rightBoundaryVertices.push_back(pair(vertices.size() - 1, grid.height - 0.5));
                }
                if (isEdgeValid(grid, vertices[rightBoundaryVertices.back().first].position, vertices[bottomBoundaryVertices[0].first].position)) {
                    vertices[rightBoundaryVertices.back().first].neighbors.push_back(bottomBoundaryVertices[0].first);
                    vertices[bottomBoundaryVertices[0].first].neighbors.push_back(rightBoundaryVertices.back().first);
                }
            }

            // bottom
            if (bottomBoundaryVertices.size() > 0) {
                for (int i =0; i < bottomBoundaryVertices.size() - 1; i++) {
                    if (isEdgeValid(grid, vertices[bottomBoundaryVertices[i].first].position, vertices[bottomBoundaryVertices[i + 1].first].position)) {
                        vertices[bottomBoundaryVertices[i].first].neighbors.push_back(bottomBoundaryVertices[i + 1].first);
                        vertices[bottomBoundaryVertices[i + 1].first].neighbors.push_back(bottomBoundaryVertices[i].first);
                    }
                }
            }

            // bottom left corner
            if (grid.cells[0][0] == 0) {
                if (vertices[leftBoundaryVertices[0].first].position.y != 0.5) {
                    VoronoiVertex corner;
                    corner.position = Point{0.5, 0.5};
                    corner.index = vertices.size();
                    vertices.push_back(corner);
                    if (isEdgeValid(grid, vertices[leftBoundaryVertices[0].first].position, corner.position)) {
                        vertices.back().neighbors.push_back(leftBoundaryVertices[0].first);
                        vertices[leftBoundaryVertices[0].first].neighbors.push_back(vertices.size() - 1);
                    }
                    leftBoundaryVertices.insert(leftBoundaryVertices.begin(), pair(corner.index, 0.5));
                }
                if (isEdgeValid(grid, vertices[bottomBoundaryVertices.back().first].position, vertices[leftBoundaryVertices[0].first].position)) {
                    vertices[bottomBoundaryVertices.back().first].neighbors.push_back(leftBoundaryVertices[0].first);
                    vertices[leftBoundaryVertices[0].first].neighbors.push_back(bottomBoundaryVertices.back().first);
                }
            }
        }

    public:
        VoronoiDiagram(Grid& grid, vector<Point>& centers) {
            K::Iso_rectangle_2 bbox(0.5, 0.5, grid.width - 0.5, grid.height - 0.5);
            this->bbox = bbox;
            auto graph_start = chrono::high_resolution_clock::now();
            buildVoronoiGraph(grid, centers);
            auto graph_end = chrono::high_resolution_clock::now();
            auto voronoi_start = chrono::high_resolution_clock::now();
            buildVoronoiVertices(grid);
            auto voronoi_end = chrono::high_resolution_clock::now();
        }

        Delaunay getGraph() {
            return graph;
        }

        vector<VoronoiVertex> getVertices() {
            return vertices;
        }

        void update(Grid& grid, Point& remove, Point& insert) {
            // update graph
            Delaunay::Vertex_handle vh_remove = graph.nearest_vertex(K::Point_2(remove.x, remove.y));
            graph.remove(vh_remove);
            graph.insert(K::Point_2(insert.x, insert.y));
            // rebuild voronoi
            vertices.clear();
            buildVoronoiVertices(grid);
        }

        void insert(Grid& grid, Point& insert) {
            // update graph
            auto graph_start = chrono::high_resolution_clock::now();
            graph.insert(K::Point_2(insert.x, insert.y));
            auto graph_end = chrono::high_resolution_clock::now();
            // rebuild voronoi
            auto voronoi_start = chrono::high_resolution_clock::now();
            vertices.clear();
            buildVoronoiVertices(grid);
            auto voronoi_end = chrono::high_resolution_clock::now();
        }

        void remove(Grid& grid, Point& remove) {
            // update graph
            Delaunay::Vertex_handle vh_remove = graph.nearest_vertex(K::Point_2(remove.x, remove.y));
            graph.remove(vh_remove);
            // rebuild voronoi
            vertices.clear();
            buildVoronoiVertices(grid);
        }

};

    