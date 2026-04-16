#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "algorithm/obstacleGenerator.hpp"
#include "algorithm/occupancyMapGenerator.hpp"
#include "algorithm/_index.hpp"
#include "benchmark/astar.hpp"
#include "benchmark/dijkstra.hpp"
#include "benchmark/bfs.hpp"
using namespace std;
using namespace chrono;

int main(int argc, char** argv) {

    // controlled variables
    int seed = atoi(argv[1]);
    int width = atoi(argv[2]);
    int height = atoi(argv[3]);
    int numObstacle = atoi(argv[4]);
    int v_max = atoi(argv[5]);
    int radius_min = atoi(argv[6]);
    int radius_max = atoi(argv[7]);
    Grid grid = {(double)width, (double)height, vector<vector<int>>(width, vector<int>(height, 0))};
    Point start = {0.5, 0.5};
    Point goal = {width - 0.5, height - 0.5};

    // generate obstacle map
    ObstacleGenerator generator(seed, numObstacle, v_max, radius_min, radius_max, grid);
    vector<Obstacle> obstacles = generator.generateObstacles();

    // generate occupancy map
    OccupancyMapGenerator occupancyMap(grid);
    Grid map = occupancyMap.generateOccupancyMap(obstacles, grid);

    // check if feasible
    DijkstraGrid path(map, start, goal);
    if (path.getPath().empty()) {
        cout << "No path exists \n";
        return 0;
    }



    // Algorithms

    // Hybrid Voronoi with A*
    HybridVoronoiA hybridVoronoiA(map, obstacles, start, goal);
    vector<Point> hybridVronoiAPath = hybridVoronoiA.getPath();
    vector<VoronoiVertex> graph = hybridVoronoiA.getGraph();
    cout << "Hybrid Voronoi with A*: " << hybridVoronoiA.getTime() << "ms | Length: " << hybridVoronoiA.getLength() << "\n";

    // A* Grid
    AStarGrid aStarGrid(map, start, goal);
    vector<Point> aStarGridPath = aStarGrid.getPath();
    cout << "A* Grid: " << aStarGrid.getTime() << "ms | Length: " << aStarGrid.getLength() << "\n";

    // Dijkstra
    DijkstraGrid dijkstraGrid(map, start, goal);
    vector<Point> dijkstraPath = dijkstraGrid.getPath();
    cout << "Dijkstra: " << dijkstraGrid.getTime() << "ms | Length: " << dijkstraGrid.getLength() << "\n";

    // BFS
    BFSGrid bfs(map, start, goal);
    vector<Point> bfsPath = bfs.getPath();
    cout << "Breadth First Search: " << bfs.getTime() << "ms | Length: " << bfs.getLength() << "\n";




    // Output for plotting

    ofstream file("obstacles.csv");
    for (int i = 0; i < obstacles.size(); i++) {
        for (int j = 0; j < obstacles[i].vertices.size(); j++) {
            file << obstacles[i].vertices[j].x << "," << obstacles[i].vertices[j].y << "\n";
        }
        if (obstacles[i].vertices.size() > 0) {
            file << "END\n";
        }
    }
    file.close();

    ofstream file2("occupancy_map.csv");
    for (int y = 0; y < map.height; y++) {
        for (int x = 0; x < map.width; x++) {
            file2 << map.cells[y][x] << ",";
        }
        file2 << "\n";
    }
    file2.close();

    ofstream file3("voronoi_vertices.csv");
    for (int i = 0; i < graph.size(); i++) {
        file3 << graph[i].position.x << "," << graph[i].position.y << "," << "[";
        for (int j = 0; j < graph[i].neighbors.size(); j++) {
            file3 << graph[i].neighbors[j];
            if (j < graph[i].neighbors.size() - 1) {
                file3 << ";";
            }
        }        file3 << "]\n";
    }
    file3.close();

    ofstream file4("hybridVoronoiAPath.csv");
    for (int i = 0; i < hybridVronoiAPath.size(); i++) {
        file4 << hybridVronoiAPath[i].x << "," << hybridVronoiAPath[i].y << "\n";
    }
    file4.close();

    ofstream file5("aStarGridPath.csv");
    for (int i = 0; i < aStarGridPath.size(); i++) {
        file5 << aStarGridPath[i].x << "," << aStarGridPath[i].y << "\n";
    }
    file5.close();

    ofstream file6("dijkstraPath.csv");
    for (int i = 0; i < dijkstraPath.size(); i++) {
        file6 << dijkstraPath[i].x << "," << dijkstraPath[i].y << "\n";
    }
    file6.close();

    ofstream file7("bfsPath.csv");
    for (int i = 0; i < bfsPath.size(); i++) {
        file7 << bfsPath[i].x << "," << bfsPath[i].y << "\n";
    }
    file7.close();

    return 0;

}