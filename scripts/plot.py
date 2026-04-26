import matplotlib.pyplot as plt
import numpy as np

# Load polygons
polygons = []
with open("obstacles.csv") as f:
    current = []
    for line in f:
        if line.strip() == "END":
            polygons.append(current)
            current = []
        else:
            x, y = map(float, line.split(","))
            current.append((x, y))

# Load occupancy map
occupancy_map = []
with open("occupancy_map.csv") as f:
    for line in f:
        row = [int(v) for v in line.strip().split(",") if v != ""]
        occupancy_map.append(row)
occupancy = np.array(occupancy_map)

# Load cluster centers
cluster_centers = []
with open("cluster_centers.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        cluster_centers.append((x,y))


# Load voronoi vertices
# voronoi_vertices = []
# with open("voronoi_vertices.csv") as f:
#     for line in f:
#         x, y = map(float, line.strip().split(","))
#         voronoi_vertices.append((x,y))

voronoi_vertices = []
with open("voronoi_vertices.csv") as f:
    for line in f:
        line = line.strip()

        # Split into parts
        parts = line.split(",", 2)
        x = float(parts[0])
        y = float(parts[1])

        # Extract neighbors inside [...]
        neighbors_str = parts[2].strip()[1:-1]  # remove [ ]
        if neighbors_str:
            neighbors = list(map(int, neighbors_str.split(";")))
        else:
            neighbors = []

        voronoi_vertices.append({
            "pos": (x, y),
            "neighbors": neighbors
        })

# Load Hybrid Voronoi A* path
hybrid_path = []
with open("hybrid_path.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        hybrid_path.append((x, y))

# A* path
astar_path = []
with open("astar_path.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        astar_path.append((x, y))

# # A* path
# dijkstraPath = []
# with open("dijkstraPath.csv") as f:
#     for line in f:
#         x, y = map(float, line.strip().split(","))
#         dijkstraPath.append((x, y))

# BFS path
bfs_path = []
with open("bfs_path.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        bfs_path.append((x, y))

# Plotting
plt.imshow(
    occupancy,
    cmap="gray_r",
    origin="lower",
    extent=[0, occupancy.shape[1], 0, occupancy.shape[0]]
)

for poly in polygons:
    xs = [p[0] for p in poly] + [poly[0][0]]
    ys = [p[1] for p in poly] + [poly[0][1]]
    plt.plot(xs, ys, color="red", linewidth=1.5)

centers_x, centers_y = zip(*cluster_centers)
plt.scatter(centers_x, centers_y, c='red', marker='o', label="Cluster Centers")

# voronoi_x, voronoi_y = zip(*voronoi_vertices)
# plt.scatter(voronoi_x, voronoi_y, c='blue', marker='o', label="Voronoi Vertices")

for i, v in enumerate(voronoi_vertices):
    x1, y1 = v["pos"]

    for n in v["neighbors"]:
        x2, y2 = voronoi_vertices[n]["pos"]

        plt.plot([x1, x2], [y1, y2], linewidth=1)

xs = [v["pos"][0] for v in voronoi_vertices]
ys = [v["pos"][1] for v in voronoi_vertices]

# Hybrid Voronoi A* Path
if (len(hybrid_path) > 1):
    path_xs = [p[0] for p in hybrid_path]
    path_ys = [p[1] for p in hybrid_path]
    plt.plot(path_xs, path_ys, color="blue", linewidth=2, label="Hybrid Voronoi A*")

# A* Grid Path
if (len(astar_path) > 1):
    path_xs = [p[0] for p in astar_path]
    path_ys = [p[1] for p in astar_path]
    plt.plot(path_xs, path_ys, color="red", linewidth=2, label="A* Grid")

# # Dijkstra Path
# if (len(dijkstraPath) > 1):
#     path_xs = [p[0] for p in dijkstraPath]
#     path_ys = [p[1] for p in dijkstraPath]
#     plt.plot(path_xs, path_ys, color="green", linewidth=2, label="Dijkstra")

# BFS Path
if (len(bfs_path) > 1):
    path_xs = [p[0] for p in bfs_path]
    path_ys = [p[1] for p in bfs_path]
    plt.plot(path_xs, path_ys, color="orange", linewidth=2, label="BFS")

# Config
plt.gca().set_aspect('equal')
plt.title("Obstacle + Occupancy Map")
plt.xlim(0, occupancy.shape[1])
plt.ylim(0, occupancy.shape[0])
plt.legend()
plt.savefig("results.png", dpi=300, bbox_inches="tight")
plt.show()