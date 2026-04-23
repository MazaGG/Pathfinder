#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "algorithm/obstacleGenerator.hpp"
#include "algorithm/occupancyMapGenerator.hpp"
#include "algorithm/connectedComponentLabeler.hpp"
#include "algorithm/voronoiDiagram.hpp"
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

    // find isolated clusters
    ConnectedComponentLabeler ccl(map, obstacles);
    vector<Point> centers = ccl.getCenters();
    Grid clusters = ccl.getClusters();

    // THROW (first call to load code and data into cache)
    VoronoiDiagram throw_voronoi(map, centers);

    // generate voronoi diagram
    cout << "\nIMPROVED VORONOI RESULTS: \n";
    VoronoiDiagram voronoi(map, centers);
    Delaunay graph = voronoi.getGraph();
    vector<VoronoiVertex> vertices = voronoi.getVertices();
    cout << "\n";

    // TEST: compare with freespace CDT (not really CDT since we didn't constraint it to free space, thus expect CDT to actually take longer):
    cout << "TRIANGULATION ON VERTICES RESULTS: \n";
    vector<Point> t_vertices;
    for (int i = 0; i < obstacles.size(); i++) {
        Obstacle obstacle = obstacles[i];
        for (int j = 0; j < obstacle.vertices.size(); j++) {
            t_vertices.push_back(obstacle.vertices[j]);
        }
    }
    VoronoiDiagram cdt(map, t_vertices);
    cout << "\n";

    // TEST: add one cluster (it will show wrong in the graph since I won't update the CCL for now, I just want to test update speed)
    cout << "LOCAL CHANGES: \n";
    Point newObs = {1.0, 1.0};
    centers.push_back(newObs);
    map.cells[1][1] = 1;
    voronoi.insert(map, newObs);
    graph.clear();
    graph = voronoi.getGraph();
    vertices.clear();
    vertices = voronoi.getVertices();;
    cout << "\n";

    // TEST: complete rebuild
    cout << "COMPLETE REBUILD: \n";
    VoronoiDiagram newVoronoi(map, centers);
    cout << "\n";

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

    // ofstream file4("voronoi_vertices.csv");
    // for (auto i = graph.finite_faces_begin(); i != graph.finite_faces_end(); i++) {
    //     Delaunay::Face_handle face = i;
    //     K::Point_2 circumcenter = graph.circumcenter(face);
    //     file4 << circumcenter.x() << "," << circumcenter.y() << "\n";
    // }
    // file4.close();

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

    // ofstream file4("hybridVoronoiAPath.csv");
    // for (int i = 0; i < hybridVronoiAPath.size(); i++) {
    //     file4 << hybridVronoiAPath[i].x << "," << hybridVronoiAPath[i].y << "\n";
    // }
    // file4.close();

    // ofstream file5("aStarGridPath.csv");
    // for (int i = 0; i < aStarGridPath.size(); i++) {
    //     file5 << aStarGridPath[i].x << "," << aStarGridPath[i].y << "\n";
    // }
    // file5.close();

    // ofstream file6("dijkstraPath.csv");
    // for (int i = 0; i < dijkstraPath.size(); i++) {
    //     file6 << dijkstraPath[i].x << "," << dijkstraPath[i].y << "\n";
    // }
    // file6.close();

    // ofstream file7("bfsPath.csv");
    // for (int i = 0; i < bfsPath.size(); i++) {
    //     file7 << bfsPath[i].x << "," << bfsPath[i].y << "\n";
    // }
    // file7.close();

    return 0;

}