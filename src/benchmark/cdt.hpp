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
        const Grid* gridPtr;  // Store grid reference for edge validation
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
                
                // Check bounds
                if (pos.x < 0 || pos.x > grid.width || 
                    pos.y < 0 || pos.y > grid.height) {
                    faceToIndex[f] = -1;
                    continue;
                }
                
                // Check if this vertex falls in an occupied cell
                int gx = (int)pos.x;
                int gy = (int)pos.y;
                if (gx >= 0 && gx < grid.width && gy >= 0 && gy < grid.height) {
                    if (grid.cells[gy][gx] == 1) {
                        faceToIndex[f] = -1;  // Skip — vertex inside obstacle!
                        continue;
                    }
                }
                
                VoronoiVertex v;
                v.position = pos;
                v.index = idx;
                vertices.push_back(v);
                faceToIndex[f] = idx++;
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
            if (vertices.empty() || !gridPtr) return;
            
            const double INF = numeric_limits<double>::max();
            
            // Helper: Find nearest vertex that has line-of-sight from a point
            // If none found, do a grid-based BFS from the point until we find one
            auto findReachableVertex = [&](const Point& from, set<int>& visited) -> int {
                // First try: direct line of sight to any vertex
                int bestIdx = -1;
                double bestDist = INF;
                for (int i = 0; i < vertices.size(); i++) {
                    if (visited.count(i)) continue;
                    if (isEdgeValid(*gridPtr, from, vertices[i].position)) {
                        double d = distance(from, vertices[i].position);
                        if (d < bestDist) { bestDist = d; bestIdx = i; }
                    }
                }
                if (bestIdx != -1) return bestIdx;
                
                // No direct line of sight — BFS on grid from 'from' until we find one
                vector<vector<bool>> gridVisited(gridPtr->height, vector<bool>(gridPtr->width, false));
                queue<pair<int,int>> q;
                
                int sx = (int)from.x, sy = (int)from.y;
                if (sx < 0 || sx >= gridPtr->width || sy < 0 || sy >= gridPtr->height) return -1;
                
                q.push({sx, sy});
                gridVisited[sy][sx] = true;
                
                while (!q.empty()) {
                    auto [cx, cy] = q.front(); q.pop();
                    
                    // Check if any vertex is reachable from this cell
                    Point cellPt = {(double)cx, (double)cy};
                    for (int i = 0; i < vertices.size(); i++) {
                        if (visited.count(i)) continue;
                        if (isEdgeValid(*gridPtr, cellPt, vertices[i].position)) {
                            return i;
                        }
                    }
                    
                    // Expand neighbors
                    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
                    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
                    for (int k = 0; k < 8; k++) {
                        int nx = cx + dx[k], ny = cy + dy[k];
                        if (nx < 0 || nx >= gridPtr->width || ny < 0 || ny >= gridPtr->height) continue;
                        if (gridPtr->cells[ny][nx] == 1) continue;
                        if (gridVisited[ny][nx]) continue;
                        gridVisited[ny][nx] = true;
                        q.push({nx, ny});
                    }
                }
                return -1;
            };
            
            set<int> usedStartVertices, usedGoalVertices;
            
            int startIdx = findReachableVertex(start, usedStartVertices);
            int goalIdx = findReachableVertex(goal, usedGoalVertices);
            
            if (startIdx == -1 || goalIdx == -1) return;
            
            usedStartVertices.insert(startIdx);
            usedGoalVertices.insert(goalIdx);
            
            // Iteratively try to find a path
            while (true) {
                // A* from startIdx to goalIdx
                vector<double> gScore(vertices.size(), INF);
                vector<int> cameFrom(vertices.size(), -1);
                vector<bool> closed(vertices.size(), false);
                priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> openList;
                
                gScore[startIdx] = distance(start, vertices[startIdx].position);
                openList.push({gScore[startIdx] + distance(vertices[startIdx].position, goal), startIdx});
                
                bool found = false;
                int bestNode = startIdx;
                double bestDistToGoal = distance(vertices[startIdx].position, goal);
                
                while (!openList.empty()) {
                    int current = openList.top().second;
                    openList.pop();
                    
                    if (closed[current]) continue;
                    closed[current] = true;
                    
                    // Track closest node to goal
                    double d = distance(vertices[current].position, goal);
                    if (d < bestDistToGoal) { bestDistToGoal = d; bestNode = current; }
                    
                    if (current == goalIdx) { found = true; break; }
                    
                    for (int neighbor : vertices[current].neighbors) {
                        if (closed[neighbor]) continue;
                        if (!isEdgeValid(*gridPtr, vertices[current].position, vertices[neighbor].position)) continue;
                        
                        double edgeCost = distance(vertices[current].position, vertices[neighbor].position);
                        double tentativeG = gScore[current] + edgeCost;
                        if (tentativeG < gScore[neighbor]) {
                            gScore[neighbor] = tentativeG;
                            cameFrom[neighbor] = current;
                            double fScore = tentativeG + distance(vertices[neighbor].position, goal);
                            openList.push({fScore, neighbor});
                        }
                    }
                }
                
                if (found) {
                    // Reconstruct path
                    path.clear();
                    path.push_back(start);
                    
                    int current = goalIdx;
                    vector<Point> reversed;
                    while (current != -1) {
                        reversed.push_back(vertices[current].position);
                        current = cameFrom[current];
                    }
                    for (int i = reversed.size() - 1; i >= 0; i--) path.push_back(reversed[i]);
                    path.push_back(goal);
                    return;
                }
                
                // No path — decide whether to change start or goal
                double distToStart = distance(vertices[bestNode].position, start);
                double distToGoal = distance(vertices[bestNode].position, goal);
                
                if (distToStart < distToGoal) {
                    // Best node is closer to start — try a different start vertex
                    int newStart = findReachableVertex(start, usedStartVertices);
                    if (newStart == -1) break;
                    usedStartVertices.insert(newStart);
                    startIdx = newStart;
                } else {
                    // Best node is closer to goal — try a different goal vertex
                    int newGoal = findReachableVertex(goal, usedGoalVertices);
                    if (newGoal == -1) break;
                    usedGoalVertices.insert(newGoal);
                    goalIdx = newGoal;
                }
            }
        }
    
    public:
        CDTplanner () {};

        void run(const Grid& grid, const vector<Obstacle>& obstacles, const Point& start, const Point& goal) {
            gridPtr = &grid;  // Store reference for edge validation
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