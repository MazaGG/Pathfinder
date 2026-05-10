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
#include "benchmark/dstar.hpp"
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
    ofstream file0("output/_results.csv");

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
        ofstream file1("output/obstacles-" + std::to_string(frame) + ".csv");
        for (int i = 0; i < obstacles.size(); i++) {
            for (int j = 0; j < obstacles[i].vertices.size(); j++) {
                file1 << obstacles[i].vertices[j].x << "," << obstacles[i].vertices[j].y << "\n";
            }
            if (obstacles[i].vertices.size() > 0) {
                file1 << "END\n";
            }
        }
        file1.close();

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
        FloodFill flood(map, start);
        Grid reachable = flood.getGrid();

        cout << "Checking if a path exists..." << endl;
        // check if generated map is feasible
        if (reachable.cells[(int)goal.y][(int)goal.x] == 1 || reachable.cells[(int)start.y][(int)start.x] == 1) {
            cout << "\nNO PATH EXISTS\n";
            frame++;
            continue;
        }



        // ALGORITHMS

        // Hybrid Voronoi A*
        HybridVoronoiA hybrid;
        hybrid.run(reachable, centers, start, goal);
        vector<VoronoiVertex> vertices = hybrid.getVertices();
        vector<Point> hybrid_path = hybrid.getPath();
        cout << "\nHYBRID VORONOI A*: \n";
        cout << "time: " << hybrid.getTime() << " ms\n";
        cout << "length: " << hybrid.getLength() << " units\n";

        // Dijkstra
        Dijkstra djk;
        djk.run(reachable, start, goal);
        vector<Point> djk_path = djk.getPath();
        cout << "\nDIJKSTRA*: \n";
        cout << "time: " << djk.getTime() << " ms\n";
        cout << "length: " << djk.getLength() << " units\n"; 

        // A*
        Astar astar;
        astar.run(reachable, start, goal);
        vector<Point> astar_path = astar.getPath();
        cout << "\nA*: \n";
        cout << "time: " << astar.getTime() << " ms\n";
        cout << "length: " << astar.getLength() << " units\n"; 

        // // D*
        // vector<Point> dstar_path;
        // if (frame == 0) {
        //     dstar_path = dstar.getPath(reachable, start, goal);
        // } 
        // else {
        //     dstar_path = dstar.updatePath(reachable, start, goal);
        // }
        // cout << "\nD*: \n";
        // cout << "time: " << dstar.getTime() << " ms\n";
        // cout << "length: " << dstar.getLength() << " units\n"; 

        // CDT
        CDTplanner cdt;
        cdt.run(reachable, obstacles, start, goal);
        // vector<VoronoiVertex> vertices = cdt.getVertices();
        vector<Point> cdt_path = cdt.getPath();
        cout << "\nCDT: \n";
        cout << "time: " << cdt.getTime() << " ms\n";
        cout << "length: " << cdt.getLength() << " units\n";        

        // Summary Output
        file0 << frame << "," << hybrid.getTime() << "," << hybrid.getLength() << "," << djk.getTime() << "," << djk.getLength() << "," << astar.getTime() << "," << astar.getLength() << "," << cdt.getTime() << "," << cdt.getLength() << endl;

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

        ofstream file9("output/cdt_path-" + std::to_string(frame) + ".csv");
        for (int i = 0; i <cdt_path.size(); i++) {
            file9 << cdt_path[i].x << "," << cdt_path[i].y << "\n";
        }
        file9.close();

        // ofstream file10("output/dstar_path-" + std::to_string(frame) + ".csv");
        // for (int i = 0; i <dstar_path.size(); i++) {
        //     file10 << dstar_path[i].x << "," << dstar_path[i].y << "\n";
        // }
        // file10.close();

        frame++;
    }

    return 0;
}