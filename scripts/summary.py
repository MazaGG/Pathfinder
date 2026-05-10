import csv
import os

# Read CSV manually
rows = []
with open("output/_results.csv", "r") as f:
    reader = csv.reader(f)
    for row in reader:
        rows.append(row)

# First row looks like: frame, hybrid_time, hybrid_length, dijkstra_time, dijkstra_length, astar_time, astar_length, cdt_time, cdt_length
# But without a header, we need to map columns
# 0=frame, 1=hybrid_time, 2=hybrid_length, 3=dijkstra_time, 4=dijkstra_length, 5=astar_time, 6=astar_length, 7=cdt_time, 8=cdt_length

algorithms = [
    ("Hybrid Voronoi A*", 1, 2),
    ("Dijkstra", 3, 4),
    ("A*", 5, 6),
    ("CDT-based", 7, 8),
]

# Calculate averages
results = []
dijkstra_lengths = [float(r[4]) for r in rows]
dijkstra_avg_length = sum(dijkstra_lengths) / len(dijkstra_lengths)

for name, time_idx, length_idx in algorithms:
    times = [float(r[time_idx]) for r in rows]
    lengths = [float(r[length_idx]) for r in rows]
    
    avg_time = sum(times) / len(times)
    avg_length = sum(lengths) / len(lengths)
    
    if name != "Dijkstra":
        percent_error = ((avg_length - dijkstra_avg_length) / dijkstra_avg_length) * 100
    else:
        percent_error = 0
    
    results.append({
        "Algorithm": name,
        "Avg_Time_ms": round(avg_time, 3),
        "Avg_Length": round(avg_length, 3),
        "Percent_Error_Length": round(percent_error, 3)
    })

# Write output
os.makedirs("output", exist_ok=True)
with open("output/_summary.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["Algorithm", "Avg_Time_ms", "Avg_Length", "Percent_Error_Length"])
    writer.writeheader()
    writer.writerows(results)