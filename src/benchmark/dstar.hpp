#pragma once
#include <vector>
#include <queue>
#include <limits>
#include <set>
#include <map>
#include <cmath>
#include "../helpers/struct.hpp"
#include "../helpers/function.hpp"
using namespace std;
using namespace chrono;

class Dstar {

    private:
        int searchCount = 0;
        Grid oldGrid;
        vector<Point> path;
        vector<vector<DstarNode>> dstarGrid;
        priority_queue<DstarNode, vector<DstarNode>, greater<DstarNode>> openList;
        vector<pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
        double time;
        double length;

        // when tracing path, check all neighbors for changes (old cell != new cell)

        void tracePath(const Grid& newGrid, const Point& start, const Point& goal) {
            path.push_back(start);
            DstarNode& startNode = dstarGrid[(int)start.y][(int)start.x];
            bool changes = false;

            for (int i = 0; i < directions.size(); i++) {
                int nx = startNode.x + directions[i].first;
                int ny = startNode.y + directions[i].second;

                if (nx < 0 || ny < 0 || nx >= oldGrid.width || ny >= oldGrid.height) {
                    continue;
                }

                DstarNode& neighbor = dstarGrid[ny][nx];
                neighbor.x = nx;
                neighbor.y = ny;
                if (oldGrid.cells[ny][nx] != newGrid.cells[ny][nx]) {
                    oldGrid.cells[ny][nx] = newGrid.cells[ny][nx];
                    changeNode(neighbor);
                    changes = true;
                }
            }
            if (changes) {
                findPath(oldGrid, start);
                changes = false;
            }

            for (DstarNode i = dstarGrid[startNode.parentY][startNode.parentX]; i.parentX != -1 && i.parentY != -1; i = dstarGrid[i.parentY][i.parentX]) {

                for (int j = 0; j < directions.size(); j++) {
                    int nx = i.x + directions[j].first;
                    int ny = i.y + directions[j].second;

                    if (nx < 0 || ny < 0 || nx >= oldGrid.width || ny >= oldGrid.height) {
                        continue;
                    }

                    DstarNode& neighbor = dstarGrid[ny][nx];
                    neighbor.x = nx;
                    neighbor.y = ny;
                    if (oldGrid.cells[ny][nx] != newGrid.cells[ny][nx]) {
                        // cout << "Blocked cell: (" << nx << "," << ny << ") | From: (" << i.x << "," << i.y << ")\n" << endl;
                        // cout << "Free cell: (" << nx << "," << ny << ") | From: (" << i.x << "," << i.y << ")\n" << endl;
                        oldGrid.cells[ny][nx] = newGrid.cells[ny][nx];
                        changeNode(neighbor);
                        changes = true;
                    }
                }
                if (changes) {
                    findPath(oldGrid, {(double)i.x, (double)i.y});
                    i = dstarGrid[i.y][i.x];
                    changes = false;
                }
                path.push_back(Point{(double)i.x, (double)i.y});

            }
            path.push_back(goal);
        }

        void findPath(const Grid& grid, const Point& start) {
            while (!openList.empty()) {
                searchCount++;
                DstarNode top = openList.top();
                DstarNode& current = dstarGrid[top.y][top.x];
                openList.pop();

                cout << "Current node: (" << current.x << "," << current.y << ") | H-score: " << current.hscore << " | K-score: " << current.kscore << "\n" << endl;

                if (current.kscore == current.hscore) {
                    for (int i = 0; i < directions.size(); i++) {
                        int nx = current.x + directions[i].first;
                        int ny = current.y + directions[i].second;

                        if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
                            continue;
                        }
                        if (grid.cells[ny][nx] == 1) {
                            continue;
                        }
                        if (directions[i].first != 0 && directions[i].second != 0) {
                            if (grid.cells[current.y][nx] == 1 && grid.cells[ny][current.x] == 1) {
                                continue;
                            }
                        }

                        DstarNode& neighbor = dstarGrid[ny][nx];
                        neighbor.x = nx;
                        neighbor.y = ny;
                        double cost = numeric_limits<double>::infinity();
                        if (grid.cells[ny][nx] == 0 || grid.cells[current.y][current.x] == 0) {
                            cost = distance(nx, ny, current.x, current.y);
                        }
                        double hscore = dstarGrid[current.y][current.x].hscore + cost;
                        // not yet considered OR changes in parent OR current is a better parent for neighbor
                        if (neighbor.isNew || (neighbor.parentX == current.x && neighbor.parentY == current.y && neighbor.hscore != hscore) || (neighbor.parentX != current.x && neighbor.parentY != current.y && neighbor.hscore > hscore)) {
                            neighbor.hscore = hscore;
                            neighbor.parentX = current.x;
                            neighbor.parentY = current.y;
                            if (hscore < neighbor.kscore) {
                                neighbor.kscore = hscore;
                            }
                            cout << "Low State: (" << neighbor.x << "," << neighbor.y << ")\n" << endl;
                            openList.push(neighbor);
                        }
                    }
                }
                else if (current.kscore < current.hscore) {
                    for (int i = 0; i < directions.size(); i++) {
                        int nx = current.x + directions[i].first;
                        int ny = current.y + directions[i].second;

                        if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
                            continue;
                        }
                        if (directions[i].first != 0 && directions[i].second != 0) {
                            if (grid.cells[current.y][nx] == 1 && grid.cells[ny][current.x] == 1) {
                                continue;
                            }
                        }

                        DstarNode& neighbor = dstarGrid[ny][nx];
                        neighbor.x = nx;
                        neighbor.y = ny;
                        double cost = numeric_limits<double>::infinity();
                        if (grid.cells[ny][nx] == 0 && grid.cells[current.y][current.x] == 0) {
                            cost = distance(nx, ny, current.x, current.y);
                        }
                        double hscore = dstarGrid[current.y][current.x].hscore + cost;
                        // find better parent path
                        // not yet considered OR changes in parent
                        if (neighbor.isNew || (neighbor.parentX == current.x && neighbor.parentY == current.y && neighbor.hscore != hscore)) {
                            neighbor.hscore = hscore;
                            neighbor.parentX = current.x;
                            neighbor.parentY = current.y;
                            if (hscore < neighbor.kscore) {
                                neighbor.kscore = hscore;
                            }
                            cout << "Raise State (new OR changes in parent): (" << neighbor.x << "," << neighbor.y << ")\n" << endl;
                            openList.push(neighbor);
                        }
                        // current is a better parent for neighbor
                        else if ((neighbor.parentX != current.x || neighbor.parentY != current.y) && neighbor.hscore > hscore) {
                            neighbor.hscore = hscore;
                            neighbor.parentX = current.x;
                            neighbor.parentY = current.y;
                            neighbor.kscore = hscore;
                            cout << "Raise State (current is better parent): (" << neighbor.x << "," << neighbor.y << ")\n" << endl;
                            openList.push(neighbor);
                        }
                        // neighbor is a better parent for current
                        else if ((neighbor.parentX != current.x || neighbor.parentY != current.y) && dstarGrid[current.y][current.x].hscore > neighbor.hscore + cost) {
                            hscore = neighbor.hscore + cost;
                            current.hscore = hscore;
                            current.parentX = neighbor.x;
                            current.parentY = neighbor.y;
                            current.kscore = hscore;
                            cout << "Raise State (neighbor is better parent): (" << neighbor.x << "," << neighbor.y << ")\n" << endl;
                            openList.push(current);
                        }
                    }
                }

                current.isNew = false;
                if (current.x == (int)start.x && current.y == (int)start.y) {
                    break;
                }
            }

            if (openList.empty()) {
                cout << "Emptied the openList\n" << endl;
            }
            cout << "Exited findPath | Queue size: "  << openList.size() << "\n" << endl;
        }

        void changeNode(DstarNode& change) {
            change.hscore = numeric_limits<double>::infinity();
            if (oldGrid.cells[change.y][change.x] == 0) {
                for (int i = 0; i < directions.size(); i++) {
                    int nx = change.x + directions[i].first;
                    int ny = change.y + directions[i].second;

                    if (nx < 0 || ny < 0 || nx >= oldGrid.width || ny >= oldGrid.height) {
                        continue;
                    }

                    DstarNode& neighbor = dstarGrid[ny][nx];
                    neighbor.x = nx;
                    neighbor.y = ny;

                    if (directions[i].first != 0 && directions[i].second != 0) {
                        if (oldGrid.cells[change.y][nx] == 1 && oldGrid.cells[ny][change.x] == 1) {
                            openList.push(neighbor);
                            continue;
                        }
                    }

                    double cost = numeric_limits<double>::infinity();
                    if (oldGrid.cells[ny][nx] == 0) {
                        cost = distance(nx, ny, change.x, change.y);
                    }
                    if (dstarGrid[change.y][change.x].hscore > neighbor.hscore + cost) {
                        double hscore = neighbor.hscore + cost;
                        change.hscore = hscore;
                        change.parentX = neighbor.x;
                        change.parentY = neighbor.y;
                        if (hscore < change.kscore) {
                            change.kscore = hscore;
                        }
                    }
                    openList.push(neighbor);
                }
            }
            else {
                for (int i = 0; i < directions.size(); i++) {
                    int nx = change.x + directions[i].first;
                    int ny = change.y + directions[i].second;

                    if (nx < 0 || ny < 0 || nx >= oldGrid.width || ny >= oldGrid.height) {
                        continue;
                    }

                    DstarNode& neighbor = dstarGrid[ny][nx];
                    neighbor.x = nx;
                    neighbor.y = ny;

                    if (neighbor.parentX == change.x && neighbor.parentY == change.y) {
                        neighbor.hscore = change.hscore;
                    }
                    openList.push(neighbor);
                }
            }
            openList.push(change);

            // findPath(oldGrid, Point{(double)start.x, (double)start.y});
        }

    public:
        Dstar() {}

        vector<Point> getPath(const Grid& grid, const Point& start, const Point& goal) {
            auto start_time = high_resolution_clock::now();
            this->dstarGrid.resize(grid.height, vector<DstarNode>(grid.width));
            this->oldGrid = grid;
            dstarGrid[(int)goal.y][(int)goal.x] = DstarNode{(int)goal.x, (int)goal.y, -1, -1, 0, 0, true};
            openList.push(dstarGrid[(int)goal.y][(int)goal.x]);
            findPath(oldGrid, start);
            cout << "Nodes Searched: " << searchCount << "\n" << endl;
            tracePath(oldGrid, start, goal);
            auto end_time = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(end_time - start_time).count();
            this->length = computePathLength(path);
            return path;
        }

        vector<Point> updatePath(const Grid& newGrid, const Point& start, const Point& goal) {
            auto start_time = high_resolution_clock::now();
            this->path.clear();
            tracePath(newGrid, start, goal);
            auto end_time = high_resolution_clock::now();
            this->time = duration_cast<milliseconds>(end_time - start_time).count();
            this->length = computePathLength(path);
            return path;
        }
        double getTime() { return time; }
        double getLength() { return length; }
};