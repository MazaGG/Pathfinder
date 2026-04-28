#pragma once
#include <vector>
#include <chrono>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
using namespace std;
using namespace chrono;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;

class CDTplanner {

    private:

        CDT cdt;
        vector<Point> path;
        vector<VoronoiVertex> vertices;
        double time;
        double length;

        void buildGraph(const Grid& grid, const vector<Obstacle>& obstacles) {
            // 1. Insert all obstacle vertices as sites
            vector<CDT::Point> points;
            for (const auto& obs : obstacles) {
                for (const auto& v : obs.vertices) {
                    points.emplace_back(v.x, v.y);
                }
            }
            
            // 2. Add grid border constraints
            cdt.insert(points.begin(), points.end());
            
            double offset = 0.5;
            CDT::Point bl(offset, offset);
            CDT::Point br(grid.width - offset, offset);
            CDT::Point tr(grid.width - offset, grid.height - offset);
            CDT::Point tl(offset, grid.height - offset);
            
            cdt.insert_constraint(bl, br);  // Bottom
            cdt.insert_constraint(br, tr);  // Right
            cdt.insert_constraint(tr, tl);  // Top
            cdt.insert_constraint(tl, bl);  // Left
            
            // 3. Extract Voronoi vertices from CDT faces
            map<CDT::Face_handle, int> faceToIndex;
            int idx = 0;
            
            for (auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
                auto cc = cdt.circumcenter(f);
                Point pos = {cc.x(), cc.y()};
                
                if (pos.x >= 0 && pos.x <= grid.width && 
                    pos.y >= 0 && pos.y <= grid.height) {
                    VoronoiVertex v;
                    v.position = pos;
                    v.index = idx;
                    vertices.push_back(v);
                    faceToIndex[f] = idx++;
                } else {
                    faceToIndex[f] = -1;
                }
            }
            
            // 4. Build adjacency from face neighbors
            for (auto f = cdt.finite_faces_begin(); f != cdt.finite_faces_end(); ++f) {
                int v1 = faceToIndex[f];
                if (v1 == -1) continue;
                
                for (int i = 0; i < 3; i++) {
                    auto neighborFace = f->neighbor(i);
                    if (!cdt.is_infinite(neighborFace)) {
                        int v2 = faceToIndex[neighborFace];
                        if (v2 != -1) {
                            vertices[v1].neighbors.push_back(v2);
                        }
                    }
                }
            }
        }

        void findPath(const Point& start, const Point& goal) {
            // Find nearest vertices to start and goal
            int startIdx = 0, goalIdx = 0;
            double minStart = numeric_limits<double>::infinity();
            double minGoal = numeric_limits<double>::infinity();
            
            for (int i = 0; i < vertices.size(); i++) {
                double dStart = distance(start, vertices[i].position);
                double dGoal = distance(goal, vertices[i].position);
                if (dStart < minStart) {
                    minStart = dStart;
                    startIdx = i;
                }
                if (dGoal < minGoal) {
                    minGoal = dGoal;
                    goalIdx = i;
                }
            }
            
            // A* search on the graph
            const double INF = numeric_limits<double>::max();
            vector<double> gScore(vertices.size(), INF);
            vector<int> cameFrom(vertices.size(), -1);
            vector<bool> closed(vertices.size(), false);
            
            priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> openList;
            
            gScore[startIdx] = distance(start, vertices[startIdx].position);
            openList.push({gScore[startIdx] + distance(vertices[startIdx].position, goal), startIdx});
            
            while (!openList.empty()) {
                int current = openList.top().second;
                openList.pop();
                
                if (closed[current]) continue;
                closed[current] = true;
                
                if (current == goalIdx) break;
                
                for (int neighbor : vertices[current].neighbors) {
                    if (closed[neighbor]) continue;
                    
                    double tentativeG = gScore[current] + distance(vertices[current].position, vertices[neighbor].position);
                    if (tentativeG < gScore[neighbor]) {
                        gScore[neighbor] = tentativeG;
                        cameFrom[neighbor] = current;
                        double fScore = tentativeG + distance(vertices[neighbor].position, goal);
                        openList.push({fScore, neighbor});
                    }
                }
            }
            
            // Reconstruct path
            path.clear();
            path.push_back(start);
            
            int current = goalIdx;
            vector<Point> reversed;
            while (current != -1) {
                reversed.push_back(vertices[current].position);
                current = cameFrom[current];
            }
            
            for (int i = reversed.size() - 1; i >= 0; i--) {
                path.push_back(reversed[i]);
            }
            path.push_back(goal);
        }
    
    public:

        CDTplanner(const Grid& grid, const vector<Obstacle>& obstacles, const Point& start, const Point& goal) {
            auto start_time = high_resolution_clock::now();
            buildGraph(grid, obstacles);
            findPath(start, goal);
            auto end_time = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(end_time - start_time).count();
            this->length = computePathLength(path);
        }
        
        const vector<VoronoiVertex>& getVertices() const {
            return vertices;
        }

        const vector<Point>& getPath() const {
            return path;
        }
    
        const double& getTime() const {
            return time;
        }

        const double& getLength() const {
            return length;
        }

};