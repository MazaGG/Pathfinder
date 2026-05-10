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

# Colors for each algorithm
colors = {
    "Hybrid Voronoi A*": "blue",
    "Dijkstra": "green",
    "A*": "red",
    "CDT-based": "orange",
}

# Create plots
fig, axes = plt.subplots(1, 3, figsize=(18, 5))

# Plot 1: Runtime
ax1 = axes[0]
for algo, d in data.items():
    ax1.plot(d["obstacles"], d["time"], marker='o', color=colors.get(algo, "gray"), label=algo, linewidth=2)
ax1.set_xlabel("Number of Obstacles")
ax1.set_ylabel("Average Time (ms)")
ax1.set_title("Runtime vs Obstacles")
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Path Length
ax2 = axes[1]
for algo, d in data.items():
    ax2.plot(d["obstacles"], d["length"], marker='o', color=colors.get(algo, "gray"), label=algo, linewidth=2)
ax2.set_xlabel("Number of Obstacles")
ax2.set_ylabel("Average Path Length")
ax2.set_title("Path Length vs Obstacles")
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Percent Error
ax3 = axes[2]
for algo, d in data.items():
    if algo != "Dijkstra":
        ax3.plot(d["obstacles"], d["error"], marker='o', color=colors.get(algo, "gray"), label=algo, linewidth=2)
ax3.set_xlabel("Number of Obstacles")
ax3.set_ylabel("Percent Error (%)")
ax3.set_title("Path Length Error vs Dijkstra")
ax3.legend()
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("analysis_plot.png", dpi=150)
print("Analysis plot saved to analysis_plot.png")