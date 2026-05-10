import csv
import matplotlib.pyplot as plt
import os

# Read combined data
data = {}
with open("collected/combined_summary.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        obs = int(row["Obstacles"])
        algo = row["Algorithm"]
        time = float(row["Avg_Time_ms"])
        length = float(row["Avg_Length"])
        error = float(row["Percent_Error_Length"])
        
        if algo not in data:
            data[algo] = {"obstacles": [], "time": [], "length": [], "error": []}
        
        data[algo]["obstacles"].append(obs)
        data[algo]["time"].append(time)
        data[algo]["length"].append(length)
        data[algo]["error"].append(error)

# Sort each by obstacle count
for algo in data:
    sorted_pairs = sorted(zip(data[algo]["obstacles"], data[algo]["time"], data[algo]["length"], data[algo]["error"]))
    data[algo]["obstacles"] = [x[0] for x in sorted_pairs]
    data[algo]["time"] = [x[1] for x in sorted_pairs]
    data[algo]["length"] = [x[2] for x in sorted_pairs]
    data[algo]["error"] = [x[3] for x in sorted_pairs]

# Colors
colors = {
    "Hybrid Voronoi A*": "blue",
    "Dijkstra": "green",
    "A*": "red",
    "CDT-based": "orange",
}

# Markers
markers = {
    "Hybrid Voronoi A*": "o",
    "Dijkstra": "s",
    "A*": "^",
    "CDT-based": "D",
}

# Create plots
fig, axes = plt.subplots(2, 2, figsize=(14, 12))

# Plot 1: Runtime - ALL algorithms (log scale)
ax1 = axes[0][0]
for algo, d in data.items():
    ax1.plot(d["obstacles"], d["time"], marker=markers.get(algo, "x"), 
             color=colors.get(algo, "gray"), label=algo, linewidth=2, markersize=6)
ax1.set_xlabel("Number of Obstacles")
ax1.set_ylabel("Average Time (ms) - Log Scale")
ax1.set_title("Runtime vs Obstacles (All Algorithms)")
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_yscale("log")

# Plot 2: Runtime - FAST algorithms only (linear scale)
ax2 = axes[0][1]
fast_algos = ["Hybrid Voronoi A*", "CDT-based"]
for algo in fast_algos:
    if algo in data:
        d = data[algo]
        ax2.plot(d["obstacles"], d["time"], marker=markers.get(algo, "x"), 
                 color=colors.get(algo, "gray"), label=algo, linewidth=2, markersize=8)
ax2.set_xlabel("Number of Obstacles")
ax2.set_ylabel("Average Time (ms)")
ax2.set_title("Runtime vs Obstacles (Graph-Based Methods)")
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Path Length - ALL algorithms
ax3 = axes[1][0]
for algo, d in data.items():
    ax3.plot(d["obstacles"], d["length"], marker=markers.get(algo, "x"), 
             color=colors.get(algo, "gray"), label=algo, linewidth=2, markersize=6)
ax3.set_xlabel("Number of Obstacles")
ax3.set_ylabel("Average Path Length")
ax3.set_title("Path Length vs Obstacles")
ax3.legend()
ax3.grid(True, alpha=0.3)

# Plot 4: Path Length - Zoomed (exclude Dijkstra/A* if they dwarf others)
ax4 = axes[1][1]
for algo, d in data.items():
    ax4.plot(d["obstacles"], d["length"], marker=markers.get(algo, "x"), 
             color=colors.get(algo, "gray"), label=algo, linewidth=2, markersize=6)
ax4.set_xlabel("Number of Obstacles")
ax4.set_ylabel("Average Path Length")
ax4.set_title("Path Length vs Obstacles (Zoomed)")
ax4.legend()
ax4.grid(True, alpha=0.3)
# Auto-zoom: find min/max excluding outliers if needed
all_lengths = []
for algo, d in data.items():
    all_lengths.extend(d["length"])
if all_lengths:
    ax4.set_ylim(min(all_lengths) * 0.95, max(all_lengths) * 1.05)

plt.tight_layout()
plt.savefig("analysis_plot.png", dpi=150)
print("Analysis plot saved to analysis_plot.png")