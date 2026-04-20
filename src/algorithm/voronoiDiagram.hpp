#pragma once
#include <vector>
#include <cmath>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 CGALPoint;
typedef K::Ray_2 CGALRay;
typedef K::Segment_2 CGALSegment;
typedef K::Line_2 CGALLine;

class VoronoiDiagram {

    private:
        vector<VoronoiVertex> vertices;
        vector<int> borderVertices; 

        void clipEdgeToGrid(Delaunay::Face_handle f, int edge_idx, int v1, Grid& grid, const vector<pair<Point, Point>>& boundaries, vector<VoronoiVertex>& vertices, map<pair<double,double>, int>& boundaryPointToIndex) {
            
            Point source = vertices[v1].position;
            
            // Get the two points forming the Delaunay edge
            CGALPoint cgal_p1 = f->vertex((edge_idx + 1) % 3)->point();
            CGALPoint cgal_p2 = f->vertex((edge_idx + 2) % 3)->point();
            
            Point p1 = {cgal_p1.x(), cgal_p1.y()};
            Point p2 = {cgal_p2.x(), cgal_p2.y()};
            
            // The Voronoi edge is the perpendicular bisector of the Delaunay edge
            // Direction: rotate (p2 - p1) by 90 degrees
            double dx = p2.y - p1.y;
            double dy = -(p2.x - p1.x);
            
            // Normalize
            double len = sqrt(dx*dx + dy*dy);
            if (len < 1e-9) return;
            dx /= len;
            dy /= len;
            
            // Determine which direction points outward
            // The Voronoi vertex is the circumcenter of the triangle
            // The direction should point away from the triangle's third vertex
            CGALPoint cgal_p3 = f->vertex(edge_idx)->point();
            Point p3 = {cgal_p3.x(), cgal_p3.y()};
            
            // Vector from edge midpoint to p3
            Point mid = {(p1.x + p2.x) / 2, (p1.y + p2.y) / 2};
            double toP3_x = p3.x - mid.x;
            double toP3_y = p3.y - mid.y;
            
            // Check which perpendicular direction is opposite to p3
            double dot1 = dx * toP3_x + dy * toP3_y;
            
            // We want the direction that goes AWAY from p3
            if (dot1 > 0) {
                dx = -dx;
                dy = -dy;
            }
            
            // Now we have the correct outward direction
            double farDist = max(grid.width, grid.height) * 3;
            Point farPoint = {source.x + dx * farDist, source.y + dy * farDist};
            
            Point bestIntersection;
            double minDist = numeric_limits<double>::infinity();
            
            // Find intersection with grid boundaries
            for (const auto& boundary : boundaries) {
                if (segmentsIntersect(source, farPoint, boundary.first, boundary.second)) {
                    Point intersection = intersectionPoint(source, farPoint,
                                                        boundary.first, boundary.second);
                    double dist = distance(source, intersection);
                    if (dist < minDist && dist > 1e-9) {
                        minDist = dist;
                        bestIntersection = intersection;
                    }
                }
            }
            
            if (minDist < numeric_limits<double>::infinity()) {
                // Snap to grid boundary to avoid floating point issues
                bestIntersection = snapToGridBoundary(bestIntersection, grid);
                
                // Create or get boundary vertex
                auto key = make_pair(bestIntersection.x, bestIntersection.y);
                int boundaryIdx;
                
                auto it = boundaryPointToIndex.find(key);
                if (it != boundaryPointToIndex.end()) {
                    boundaryIdx = it->second;
                } else {
                    VoronoiVertex v;
                    v.position = bestIntersection;
                    vertices.push_back(v);
                    boundaryIdx = vertices.size() - 1;
                    boundaryPointToIndex[key] = boundaryIdx;
                }
                
                // Add bidirectional connection
                vertices[v1].neighbors.push_back(boundaryIdx);
                vertices[boundaryIdx].neighbors.push_back(v1);
            }
        }

        void buildVoronoiGraph(Grid& grid, vector<Point>& centers) {
            // Insert points
            vector<CGALPoint> points;
            for (int i = 0; i < centers.size(); i++) {
                points.emplace_back(centers[i].x, centers[i].y);
            }

            Delaunay dt;
            dt.insert(points.begin(), points.end());
            map<Delaunay::Face_handle, int> faceToIndex;
            map<pair<double,double>, int> boundaryPointToIndex;
            
            // Helper to check if point is inside grid
            auto isInsideGrid = [&](const Point& p) -> bool {
                return p.x >= 0 && p.x <= grid.width && p.y >= 0 && p.y <= grid.height;
            };
            
            // Helper to clamp point to grid
            auto clampToGrid = [&](const Point& p) -> Point {
                return {
                    max(0.0, min((double)grid.width, p.x)),
                    max(0.0, min((double)grid.height, p.y))
                };
            };

            int idx = 0;

            // Create Voronoi vertices (circumcenters) - only keep those inside grid
            for (Delaunay::Finite_faces_iterator f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
                CGALPoint cc = dt.dual(f);
                Point pos = {cc.x(), cc.y()};
                
                if (isInsideGrid(pos)) {
                    VoronoiVertex v;
                    v.position = pos;
                    vertices.push_back(v);
                    faceToIndex[f] = idx++;
                } else {
                    faceToIndex[f] = -1;  // Mark as outside
                }
            }

            // Grid boundaries for intersection testing
            vector<pair<Point, Point>> boundaries = {
                {{0, 0}, {grid.width, 0}},                    
                {{grid.width, 0}, {grid.width, grid.height}}, 
                {{grid.width, grid.height}, {0, grid.height}},
                {{0, grid.height}, {0, 0}}                    
            };

            // Connect neighbors
            for (Delaunay::Finite_faces_iterator f = dt.finite_faces_begin(); f != dt.finite_faces_end(); ++f) {
                int v1 = faceToIndex[f];
                if (v1 == -1) continue;  // Skip faces outside grid

                for (int i = 0; i < 3; i++) {
                    Delaunay::Face_handle neighbor = f->neighbor(i);
                    
                    if (!dt.is_infinite(neighbor)) {
                        int v2 = faceToIndex[neighbor];
                        if (v2 != -1) {
                            // Both vertices inside grid - normal connection
                            vertices[v1].neighbors.push_back(v2);
                        } else {
                            // Neighbor is outside grid - need to clip this edge
                            clipEdgeToGrid(f, i, v1, grid, boundaries, vertices, boundaryPointToIndex);
                        }
                    } else {
                        // Infinite edge - clip to grid
                        clipEdgeToGrid(f, i, v1, grid, boundaries, vertices, boundaryPointToIndex);
                    }
                }
            }
        }

    public:
    
        VoronoiDiagram(Grid& grid, vector<Point>& centers) {
            buildVoronoiGraph(grid, centers);
        }

        vector<VoronoiVertex> getVertices() {
            return vertices;
        }

};

    