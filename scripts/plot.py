import matplotlib.pyplot as plt
import numpy as np

polygons = []
occupancy_map = []
vor_vertices = []

# Load polygons
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
with open("occupancy_map.csv") as f:
    for line in f:
        row = [int(v) for v in line.strip().split(",") if v != ""]
        occupancy_map.append(row)
occupancy = np.array(occupancy_map)

# Load voronoi vertices
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

        vor_vertices.append({
            "pos": (x, y),
            "neighbors": neighbors
        })

# Load Hybrid Voronoi A* path
hybridVoronoiAPath = []
with open("hybridVoronoiAPath.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        hybridVoronoiAPath.append((x, y))

# A* path
aStarGridPath = []
with open("aStarGridPath.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        aStarGridPath.append((x, y))

# A* path
dijkstraPath = []
with open("dijkstraPath.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        dijkstraPath.append((x, y))

# BFS path
bfsPath = []
with open("bfsPath.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        bfsPath.append((x, y))

# RRT path
rrtPath = []
with open("rrtPath.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        rrtPath.append((x, y))

# JPS path
jpsPath = []
with open("jpsPath.csv") as f:
    for line in f:
        x, y = map(float, line.strip().split(","))
        jpsPath.append((x, y))




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

for i, v in enumerate(vor_vertices):
    x1, y1 = v["pos"]

    for n in v["neighbors"]:
        x2, y2 = vor_vertices[n]["pos"]

        plt.plot([x1, x2], [y1, y2], linewidth=1)

xs = [v["pos"][0] for v in vor_vertices]
ys = [v["pos"][1] for v in vor_vertices]

# Hybrid Voronoi A* Path
if (len(hybridVoronoiAPath) > 1):
    path_xs = [p[0] for p in hybridVoronoiAPath]
    path_ys = [p[1] for p in hybridVoronoiAPath]
    plt.plot(path_xs, path_ys, color="blue", linewidth=2, label="Hybrid Voronoi A*")

# A* Grid Path
if (len(aStarGridPath) > 1):
    path_xs = [p[0] for p in aStarGridPath]
    path_ys = [p[1] for p in aStarGridPath]
    plt.plot(path_xs, path_ys, color="red", linewidth=2, label="A* Grid")

# Dijkstra Path
if (len(dijkstraPath) > 1):
    path_xs = [p[0] for p in dijkstraPath]
    path_ys = [p[1] for p in dijkstraPath]
    plt.plot(path_xs, path_ys, color="green", linewidth=2, label="Dijkstra")

# BFS Path
if (len(bfsPath) > 1):
    path_xs = [p[0] for p in bfsPath]
    path_ys = [p[1] for p in bfsPath]
    plt.plot(path_xs, path_ys, color="orange", linewidth=2, label="BFS")

# RRT Path
if (len(rrtPath) > 1):
    path_xs = [p[0] for p in rrtPath]
    path_ys = [p[1] for p in rrtPath]
    plt.plot(path_xs, path_ys, color="yellow", linewidth=2, label="RRT")

# JPS Path
if (len(jpsPath) > 1):
    path_xs = [p[0] for p in jpsPath]
    path_ys = [p[1] for p in jpsPath]
    plt.plot(path_xs, path_ys, color="pink", linewidth=2, label="JPS")


# Config
plt.gca().set_aspect('equal')
plt.title("Obstacle + Occupancy Map")
plt.xlim(0, occupancy.shape[1])
plt.ylim(0, occupancy.shape[0])
plt.legend()
plt.savefig("results.png", dpi=300, bbox_inches="tight")
plt.show()