#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "algorithm/obstacleGenerator.hpp"
#include "algorithm/occupancyMapGenerator.hpp"
#include "algorithm/connectedComponentLabeler.hpp"
#include "algorithm/floodfill.hpp"
#include "algorithm/_index.hpp"
#include "benchmark/astar.hpp"
#include "benchmark/dijkstra.hpp"
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

    // find isolated clusters
    ConnectedComponentLabeler ccl(map, obstacles);
    vector<Point> centers = ccl.getCenters();
    Grid clusters = ccl.getClusters();

    // flood fill the map
    FloodFill flood(map, start);
    Grid reachable = flood.getGrid();

    // check if generated map is feasible
    if (reachable.cells[(int)goal.y][(int)goal.x] == 1 || map.cells[(int)start.y][(int)start.x] == 1) {
        cout << "\nNO PATH EXISTS\n";
        return 0;
    }



    // ALGORITHMS

    // Hybrid Voronoi A*
    HybridVoronoiA throw_hybrid(reachable, centers, start, goal);
    HybridVoronoiA hybrid(reachable, centers, start, goal);
    vector<VoronoiVertex> vertices = hybrid.getVertices();
    vector<Point> hybrid_path = hybrid.getPath();
    cout << "\nHYBRID VORONOI A*: \n";
    cout << "time: " << hybrid.getTime() << " ms\n";
    cout << "length: " << hybrid.getLength() << " units\n";

    // A*
    Astar throw_astar(reachable, start, goal);
    Astar astar(reachable, start, goal);
    vector<Point> astar_path = astar.getPath();
    cout << "\nA*: \n";
    cout << "time: " << astar.getTime() << " ms\n";
    cout << "length: " << astar.getLength() << " units\n"; 

    // Dijkstra
    Dijkstra throw_djk(reachable, start, goal);
    Dijkstra djk(reachable, start, goal);
    vector<Point> djk_path = djk.getPath();
    cout << "\nDIJKSTRA*: \n";
    cout << "time: " << djk.getTime() << " ms\n";
    cout << "length: " << djk.getLength() << " units\n"; 

    // // TEST: compare with freespace CDT (not really CDT since we didn't constraint it to free space, thus expect CDT to actually take longer):
    // cout << "\nTRIANGULATION ON VERTICES RESULTS: \n";
    // vector<Point> t_vertices;
    // for (int i = 0; i < obstacles.size(); i++) {
    //     Obstacle obstacle = obstacles[i];
    //     for (int j = 0; j < obstacle.vertices.size(); j++) {
    //         t_vertices.push_back(obstacle.vertices[j]);
    //     }
    // }
    // VoronoiDiagram cdt(reachable, t_vertices);
    // cout << "\n";

    // // TEST: add one cluster (it will show wrong in the graph since I won't update the CCL for now, I just want to test update speed)
    // cout << "LOCAL CHANGES: \n";
    // Point newObs = {1.0, 1.0};
    // centers.push_back(newObs);
    // map.cells[1][1] = 1;
    // voronoi.insert(map, newObs);
    // graph.clear();
    // graph = voronoi.getGraph();
    // vertices.clear();
    // vertices = voronoi.getVertices();;
    // cout << "\n";

    // // TEST: complete rebuild
    // cout << "COMPLETE REBUILD: \n";
    // VoronoiDiagram newVoronoi(map, centers);
    // cout << "\n";





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

    ofstream file3("cluster_centers.csv");
    for (int i = 0; i < centers.size(); i++) {
        file3 << centers[i].x << "," << centers[i].y << "\n";
    }
    file3.close();

    ofstream file4("voronoi_vertices.csv");
    for (int i = 0; i < vertices.size(); i++) {
        file4 << vertices[i].position.x << "," << vertices[i].position.y << "," << "[";
        for (int j = 0; j < vertices[i].neighbors.size(); j++) {
            file4 << vertices[i].neighbors[j];
            if (j < vertices[i].neighbors.size() - 1) {
                file4 << ";";
            }
        }        file4 << "]\n";
    }
    file4.close();

    ofstream file5("cluster_map.csv");
    for (int y = 0; y < clusters.height; y++) {
        for (int x = 0; x < clusters.width; x++) {
            file5 << clusters.cells[y][x] << ",";
        }
        file5 << "\n";
    }
    file5.close();

    ofstream file6("hybrid_path.csv");
    for (int i = 0; i < hybrid_path.size(); i++) {
        file6 << hybrid_path[i].x << "," << hybrid_path[i].y << "\n";
    }
    file6.close();

    ofstream file7("astar_path.csv");
    for (int i = 0; i < astar_path.size(); i++) {
        file7 << astar_path[i].x << "," << astar_path[i].y << "\n";
    }
    file7.close();

    ofstream file8("djk_path.csv");
    for (int i = 0; i <djk_path.size(); i++) {
        file8 << djk_path[i].x << "," << djk_path[i].y << "\n";
    }
    file8.close();

    return 0;

}