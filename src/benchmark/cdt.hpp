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
        // vector<vector<faceNode>> faceGrid;
        vector<VoronoiVertex> vertices;
        const Grid* gridPtr;  // Store grid reference for edge validation
        double time;
        double length;

        // void buildGraph(const Grid& grid, const vector<Obstacle>& obstacles) {
        //     vector<CDT::Point> points;
        //     for (const auto& obs: obstacles) {
        //         for (const auto& v: obs.vertices) {
        //             points.emplace_back(v.x, v.y);
        //         }
        //     }
        //     cdt.insert(points.begin(),points.end());

        //     CDT::Point bl(0.5, 0.5);
        //     CDT::Point br(grid.width - 0.5, 0.5);
        //     CDT::Point tr(grid.width - 0.5, grid.height - 0.5);
        //     CDT::Point tl(0.5, grid.height - 0.5);            
        //     cdt.insert_constraint(bl, br);  // Bottom
        //     cdt.insert_constraint(br, tr);  // Right
        //     cdt.insert_constraint(tr, tl);  // Top
        //     cdt.insert_constraint(tl, bl);  // Left
        // }

        // void findPath(const Point& start, const Point& goal) {
        //     // find what face is start located
        //     // find what face is goal located
        //     // 
        //     // initialize priority queue of <face, double>
        //     // add <start face, 0> to queue
        //     // while queue is not empty:
        //     //      current face = queue.pop
        //     //      if current face == goal face:
        //     //          path found
        //     //      for each neighbor face of current face:
        //     //          point = bisector of current and neighbor's shared edge;
        //     //          if collision to go to point:
        //     //              continue
        //     //          if the point is outside grid:
        //     //              continue
        //     //          check if we can improve parent via current
        //     //          add to queue
        // }


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
            
            // Helper: Find nearest vertex that has line-of-sight, with BFS path
            auto findReachableVertex = [&](const Point& from, set<int>& visited, vector<Point>& bfsPath) -> int {
                bfsPath.clear();
                
                // First try: direct line of sight
                int bestIdx = -1;
                double bestDist = INF;
                for (int i = 0; i < vertices.size(); i++) {
                    if (visited.count(i)) continue;
                    if (isEdgeValid(*gridPtr, from, vertices[i].position)) {
                        double d = distance(from, vertices[i].position);
                        if (d < bestDist) { bestDist = d; bestIdx = i; }
                    }
                }
                if (bestIdx != -1) {
                    bfsPath.push_back(from);
                    bfsPath.push_back(vertices[bestIdx].position);
                    return bestIdx;
                }
                
                // BFS on grid
                vector<vector<bool>> gridVisited(gridPtr->height, vector<bool>(gridPtr->width, false));
                vector<vector<pair<int,int>>> parent(gridPtr->height, vector<pair<int,int>>(gridPtr->width, {-1, -1}));
                queue<pair<int,int>> q;
                
                int sx = (int)from.x, sy = (int)from.y;
                if (sx < 0 || sx >= gridPtr->width || sy < 0 || sy >= gridPtr->height) return -1;
                
                q.push({sx, sy});
                gridVisited[sy][sx] = true;
                int foundX = -1, foundY = -1;
                
                while (!q.empty()) {
                    auto [cx, cy] = q.front(); q.pop();
                    
                    Point cellPt = {(double)cx, (double)cy};
                    for (int i = 0; i < vertices.size(); i++) {
                        if (visited.count(i)) continue;
                        if (isEdgeValid(*gridPtr, cellPt, vertices[i].position)) {
                            foundX = cx;
                            foundY = cy;
                            bestIdx = i;
                            break;
                        }
                    }
                    if (bestIdx != -1) break;
                    
                    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
                    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
                    for (int k = 0; k < 8; k++) {
                        int nx = cx + dx[k], ny = cy + dy[k];
                        if (nx < 0 || nx >= gridPtr->width || ny < 0 || ny >= gridPtr->height) continue;
                        if (gridPtr->cells[ny][nx] == 1) continue;
                        if (gridVisited[ny][nx]) continue;
                        gridVisited[ny][nx] = true;
                        parent[ny][nx] = {cx, cy};
                        q.push({nx, ny});
                    }
                }
                
                if (bestIdx != -1) {
                    // Reconstruct BFS path
                    vector<Point> reversed;
                    int cx = foundX, cy = foundY;
                    while (cx != sx || cy != sy) {
                        reversed.push_back({(double)cx, (double)cy});
                        auto [px, py] = parent[cy][cx];
                        cx = px; cy = py;
                    }
                    reversed.push_back(from);
                    
                    for (int i = reversed.size() - 1; i >= 0; i--) {
                        bfsPath.push_back(reversed[i]);
                    }
                    bfsPath.push_back(vertices[bestIdx].position);
                }
                
                return bestIdx;
            };
            
            set<int> usedStartVertices, usedGoalVertices;
            vector<Point> startBfsPath, goalBfsPath;
            
            int startIdx = findReachableVertex(start, usedStartVertices, startBfsPath);
            int goalIdx = findReachableVertex(goal, usedGoalVertices, goalBfsPath);
            
            if (startIdx == -1 || goalIdx == -1) return;
            
            usedStartVertices.insert(startIdx);
            usedGoalVertices.insert(goalIdx);
            
            while (true) {
                vector<double> gScore(vertices.size(), INF);
                vector<int> cameFrom(vertices.size(), -1);
                vector<bool> closed(vertices.size(), false);
                priority_queue<pair<double,int>, vector<pair<double,int>>, greater<pair<double,int>>> openList;
                
                gScore[startIdx] = distance(start, vertices[startIdx].position);
                openList.push({gScore[startIdx], startIdx});
                
                bool found = false;
                int bestNode = startIdx;
                double bestDistToGoal = distance(vertices[startIdx].position, goal);
                
                while (!openList.empty()) {
                    int current = openList.top().second;
                    openList.pop();
                    
                    if (closed[current]) continue;
                    closed[current] = true;
                    
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
                            openList.push({gScore[neighbor], neighbor});
                        }
                    }
                }
                
                if (found) {
                    path.clear();
                    
                    // Add start BFS path (skip last point — it's the first Voronoi vertex)
                    for (int i = 0; i < (int)startBfsPath.size() - 1; i++) {
                        path.push_back(startBfsPath[i]);
                    }
                    
                    // Add Voronoi path
                    int current = goalIdx;
                    vector<Point> reversed;
                    while (current != -1) {
                        reversed.push_back(vertices[current].position);
                        current = cameFrom[current];
                    }
                    for (int i = reversed.size() - 1; i >= 0; i--) {
                        path.push_back(reversed[i]);
                    }
                    
                    // Add goal BFS path in reverse (skip first point — it's the last Voronoi vertex)
                    reverse(goalBfsPath.begin(), goalBfsPath.end());
                    for (int i = 1; i < (int)goalBfsPath.size(); i++) {
                        path.push_back(goalBfsPath[i]);
                    }
                    
                    return;
                }
                
                double distToStart = distance(vertices[bestNode].position, start);
                double distToGoal = distance(vertices[bestNode].position, goal);
                
                if (distToStart < distToGoal) {
                    int newStart = findReachableVertex(start, usedStartVertices, startBfsPath);
                    if (newStart == -1) break;
                    usedStartVertices.insert(newStart);
                    startIdx = newStart;
                } else {
                    int newGoal = findReachableVertex(goal, usedGoalVertices, goalBfsPath);
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
            auto path_time = high_resolution_clock::now();
            findPath(start, goal);
            auto end_time = high_resolution_clock::now();
            cout << "   Build Graph: " << duration_cast<milliseconds>(path_time - start_time).count() << "ms\n";
            cout << "   Find Path: " << duration_cast<milliseconds>(end_time - path_time).count() << "ms\n";
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