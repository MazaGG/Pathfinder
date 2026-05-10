import os
import csv
import glob

output_dir = "collected"
os.makedirs(output_dir, exist_ok=True)

all_rows = []

# Walk through all artifacts
for root, dirs, files in os.walk("artifacts"):
    for file in files:
        if file == "_summary.csv":
            filepath = os.path.join(root, file)
            
            # Extract obstacle count from path (e.g., o20, o50, etc.)
            path_parts = root.split("/")
            for part in path_parts:
                if part.startswith("o") and part[1:].isdigit():
                    obstacles = int(part[1:])
                    break
            else:
                # Try to find it differently
                for part in path_parts:
                    if "-o" in part:
                        obstacles = int(part.split("-o")[-1].split("-")[0])
                        break
                    elif part.startswith("o"):
                        try:
                            obstacles = int(part[1:].split("-")[0])
                            break
                        except:
                            pass
                else:
                    continue
            
            with open(filepath, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    row["Obstacles"] = obstacles
                    all_rows.append(row)

# Write combined summary
if all_rows:
    with open(f"{output_dir}/combined_summary.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["Obstacles", "Algorithm", "Avg_Time_ms", "Avg_Length", "Percent_Error_Length"])
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"Collected {len(all_rows)} rows from artifacts")
else:
    print("No summary files found!")