import subprocess
import sys
import os

# -------------------------
# CONFIG
# -------------------------
SEED = 1
WIDTH = 100
HEIGHT = 100
NUM_OBSTACLES = 30
V_MAX = 5
RADIUS_MIN = 2
RADIUS_MAX = 6

CPP_EXEC = "./test_obstacles"   # change if needed

# -------------------------
# COMPILE (optional step)
# -------------------------
# print("Compiling C++...")

# compile_cmd = [
#     "g++",
#     "-std=c++17",
#     "main.cpp",
#     "-o",
#     "main"
# ]

# subprocess.run(compile_cmd, check=True)

# -------------------------
# RUN SIMULATION
# -------------------------
print("Running simulation...")

run_cmd = [
    CPP_EXEC,
    str(SEED),
    str(WIDTH),
    str(HEIGHT),
    str(NUM_OBSTACLES),
    str(V_MAX),
    str(RADIUS_MIN),
    str(RADIUS_MAX)
]

result = subprocess.run(run_cmd, capture_output=True, text=True)

print(result.stdout)

# -------------------------
# RUN PLOTTING
# -------------------------
print("Generating plot...")

subprocess.run(["python", "../scripts/plot.py"], check=True)

print("Done. Output saved as result.png")