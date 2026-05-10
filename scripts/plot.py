import matplotlib.pyplot as plt
import numpy as np
import os
import glob

# Find all frame files
frame_files = sorted(glob.glob("output/obstacles-*.csv"))

# Extract frame numbers from filenames
def get_frame_num(filename):
    return int(filename.split("-")[-1].replace(".csv", ""))

for frame_file in frame_files:
    frame = get_frame_num(frame_file)
    print(f"Processing frame {frame}...")
    
    # Load polygons for this frame
    polygons = []
    with open(frame_file) as f:
        current = []
        for line in f:
            if line.strip() == "END":
                polygons.append(current)
                current = []
            else:
                x, y = map(float, line.split(","))
                current.append((x, y))
    
    # Load occupancy map for this frame
    occupancy_map = []
    with open(f"output/occupancy_map-{frame}.csv") as f:
        for line in f:
            row = [int(v) for v in line.strip().split(",") if v != ""]
            occupancy_map.append(row)
    occupancy = np.array(occupancy_map)

    if os.path.exists(f"output/voronoi_vertices-{frame}.csv"):
    
        # Load voronoi vertices for this frame
        voronoi_vertices = []
        with open(f"output/voronoi_vertices-{frame}.csv") as f:
            for line in f:
                line = line.strip()
                parts = line.split(",", 2)
                x = float(parts[0])
                y = float(parts[1])
                neighbors_str = parts[2].strip()[1:-1]
                if neighbors_str:
                    neighbors = list(map(int, neighbors_str.split(";")))
                else:
                    neighbors = []
                voronoi_vertices.append({"pos": (x, y), "neighbors": neighbors})
        
        # Load hybrid path for this frame
        hybrid_path = []
        with open(f"output/hybrid_path-{frame}.csv") as f:
            for line in f:
                x, y = map(float, line.strip().split(","))
                hybrid_path.append((x, y))
        
        # Load A* path for this frame
        astar_path = []
        with open(f"output/astar_path-{frame}.csv") as f:
            for line in f:
                x, y = map(float, line.strip().split(","))
                astar_path.append((x, y))
        
        # Load Dijkstra path for this frame
        djk_path = []
        with open(f"output/djk_path-{frame}.csv") as f:
            for line in f:
                x, y = map(float, line.strip().split(","))
                djk_path.append((x, y))

        # # Load D* path for this frame
        # dstar_path = []
        # with open(f"output/dstar_path-{frame}.csv") as f:
        #     for line in f:
        #         x, y = map(float, line.strip().split(","))
        #         dstar_path.append((x, y))

        # Load CDT path for this frame
        cdt_path = []
        with open(f"output/cdt_path-{frame}.csv") as f:
            for line in f:
                x, y = map(float, line.strip().split(","))
                cdt_path.append((x, y))
        
    # ===== PLOTTING =====
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Obstacles
    ax.imshow(occupancy, cmap="gray_r", origin="lower",
            extent=[0, occupancy.shape[1], 0, occupancy.shape[0]])
    
    for poly in polygons:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.plot(xs, ys, color="red", linewidth=1)
    
    if os.path.exists(f"output/voronoi_vertices-{frame}.csv"):

        # Voronoi Diagram
        for i, v in enumerate(voronoi_vertices):
            x1, y1 = v["pos"]
            for n in v["neighbors"]:
                if n < len(voronoi_vertices):
                    x2, y2 = voronoi_vertices[n]["pos"]
                    ax.plot([x1, x2], [y1, y2], 'b-', linewidth=0.5, alpha=0.5)
        
        # Paths
        if len(hybrid_path) > 1:
            path_xs = [p[0] for p in hybrid_path]
            path_ys = [p[1] for p in hybrid_path]
            ax.plot(path_xs, path_ys, color="blue", linewidth=2, label="Hybrid Voronoi A*")
        
        if len(astar_path) > 1:
            path_xs = [p[0] for p in astar_path]
            path_ys = [p[1] for p in astar_path]
            ax.plot(path_xs, path_ys, color="red", linewidth=2, label="A* Grid")
        
        # if len(dstar_path) > 1:
        #     path_xs = [p[0] for p in dstar_path]
        #     path_ys = [p[1] for p in dstar_path]
        #     ax.plot(path_xs, path_ys, color="orange", linewidth=2, label="D* Grid")

        if len(cdt_path) > 1:
            path_xs = [p[0] for p in cdt_path]
            path_ys = [p[1] for p in cdt_path]
            ax.plot(path_xs, path_ys, color="orange", linewidth=2, label="CDT Planner")

        if len(djk_path) > 1:
            path_xs = [p[0] for p in djk_path]
            path_ys = [p[1] for p in djk_path]
            ax.plot(path_xs, path_ys, color="green", linewidth=2, label="Dijkstra")
        
    # Config
    ax.set_aspect('equal')
    ax.set_title(f"Frame {frame}")
    ax.set_xlim(0, occupancy.shape[1])
    ax.set_ylim(0, occupancy.shape[0])
    ax.legend()
    
    # Save frame
    plt.savefig(f"output/frame_{frame:03d}.png", dpi=150, bbox_inches="tight")
    plt.close()  # Close to free memory

print("All frames processed!")