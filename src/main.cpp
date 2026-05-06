#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <sys/stat.h>
#include "algorithm/obstacleGenerator.hpp"
#include "algorithm/occupancyMapGenerator.hpp"
#include "algorithm/connectedComponentLabeler.hpp"
#include "algorithm/floodfill.hpp"
#include "algorithm/_index.hpp"
#include "benchmark/astar.hpp"
#include "benchmark/dijkstra.hpp"
#include "benchmark/cdt.hpp"
using namespace std;
using namespace chrono;

int main(int argc, char** argv) {
    mkdir("output", 0777);

    // controlled variables
    cout << "\nInitializing variables..." << endl;
    int seed = atoi(argv[1]);
    int frames = atoi(argv[2]);
    int width = atoi(argv[3]);
    int height = atoi(argv[4]);
    int numObstacle = atoi(argv[5]);
    int v_max = atoi(argv[6]);
    int radius_min = atoi(argv[7]);
    int radius_max = atoi(argv[8]);
    Grid grid = {(double)width, (double)height, vector<vector<int>>(width, vector<int>(height, 0))};
    Point start = {0.5, 0.5};
    Point goal = {width - 0.5, height - 0.5};
    
    int frame = 0;
    vector<Obstacle> obstacles;
    ObstacleGenerator generator(seed, numObstacle, v_max, radius_min, radius_max, grid);

    while (frame != frames) {
        cout << "\n===== FRAME " << frame + 1 << " =====" << endl;
        
        if (frame == 0) {
            // generate obstacle map
            cout << "Generating obstacles..." << endl;
            obstacles = generator.generateObstacles();
        } else {
            // move obstacles
            cout << "Moving obstacles..." << endl;
            obstacles = generator.moveObstacles();
            grid = {(double)width, (double)height, vector<vector<int>>(width, vector<int>(height, 0))};
        }

        // generate occupancy map
        cout << "Generating occupancy map..." << endl;
        OccupancyMapGenerator occupancyMap(grid);
        Grid map = occupancyMap.generateOccupancyMap(obstacles, grid);

        // find isolated clusters
        cout << "Finding isolated clusters..." << endl;
        ConnectedComponentLabeler ccl(map, obstacles);
        vector<Point> centers = ccl.getCenters();
        Grid clusters = ccl.getClusters();

        // Obstacles
        ofstream file("output/obstacles-" + std::to_string(frame) + ".csv");
        for (int i = 0; i < obstacles.size(); i++) {
            for (int j = 0; j < obstacles[i].vertices.size(); j++) {
                file << obstacles[i].vertices[j].x << "," << obstacles[i].vertices[j].y << "\n";
            }
            if (obstacles[i].vertices.size() > 0) {
                file << "END\n";
            }
        }
        file.close();

        // Occupancy Map
        ofstream file2("output/occupancy_map-" + std::to_string(frame) + ".csv");
        for (int y = 0; y < map.height; y++) {
            for (int x = 0; x < map.width; x++) {
                file2 << map.cells[y][x] << ",";
            }
            file2 << "\n";
        }
        file2.close();

        // flood fill the map
        cout << "Checking if a path exists..." << endl;
        FloodFill flood(map, start);
        Grid reachable = flood.getGrid();

        // check if generated map is feasible
        if (reachable.cells[(int)goal.y][(int)goal.x] == 1 || map.cells[(int)start.y][(int)start.x] == 1) {
            cout << "\nNO PATH EXISTS\n";
            frame++;
            continue;
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



        // PLOTTING

        // Cluster Centers
        // ofstream file3("output/cluster_centers.csv");
        // for (int i = 0; i < centers.size(); i++) {
        //     file3 << centers[i].x << "," << centers[i].y << "\n";
        // }
        // file3.close();

        // Voronoi Vertices
        ofstream file4("output/voronoi_vertices-" + std::to_string(frame) + ".csv");
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

        // Cluster Occupancy Map
        // ofstream file5("output/cluster_map.csv");
        // for (int y = 0; y < clusters.height; y++) {
        //     for (int x = 0; x < clusters.width; x++) {
        //         file5 << clusters.cells[y][x] << ",";
        //     }
        //     file5 << "\n";
        // }
        // file5.close();

        ofstream file6("output/hybrid_path-" + std::to_string(frame) + ".csv");
        for (int i = 0; i < hybrid_path.size(); i++) {
            file6 << hybrid_path[i].x << "," << hybrid_path[i].y << "\n";
        }
        file6.close();

        ofstream file7("output/astar_path-" + std::to_string(frame) + ".csv");
        for (int i = 0; i < astar_path.size(); i++) {
            file7 << astar_path[i].x << "," << astar_path[i].y << "\n";
        }
        file7.close();

        ofstream file8("output/djk_path-" + std::to_string(frame) + ".csv");
        for (int i = 0; i <djk_path.size(); i++) {
            file8 << djk_path[i].x << "," << djk_path[i].y << "\n";
        }
        file8.close();

        // ofstream file9("cdt_path.csv");
        // for (int i = 0; i <cdt_path.size(); i++) {
        //     file9 << cdt_path[i].x << "," << cdt_path[i].y << "\n";
        // }
        // file9.close();

        frame++;
    }

    return 0;
}