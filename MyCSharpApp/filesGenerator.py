import random
import os

def generate_input_file(rows, cols, filename):
    """Generates a city grid graph and saves it to a file."""
    edges = []
    for x in range(rows):
        for y in range(cols):
            if x + 1 < rows:
                edges.append(((x, y), (x + 1, y), random.randint(1, 10)))  # Vertical edge
            if y + 1 < cols:
                edges.append(((x, y), (x, y + 1), random.randint(1, 10)))  # Horizontal edge

    # Randomly select starting point
    start = (random.randint(0, rows - 1), random.randint(0, cols - 1))

    # Randomly select target nodes
    num_targets = random.randint(2, min(5, rows * cols // 10))  # Between 2 and 5 target nodes
    targets = set()
    while len(targets) < num_targets:
        target = (random.randint(0, rows - 1), random.randint(0, cols - 1))
        if target != start:  # Ensure target is not the start node
            targets.add(target)

    targets = list(targets)

    # Write to file
    with open(filename, "w") as f:
        # Write grid dimensions
        f.write(f"{rows} {cols}\n")
        # Write edges
        for edge in edges:
            f.write(f"{edge[0][0]},{edge[0][1]} {edge[1][0]},{edge[1][1]} {edge[2]}\n") # Changed this line to match the expected format in read_input_file
        # Write starting point
        f.write(f"START {start[0]},{start[1]}\n") # Changed this line to match the expected format in read_input_file
        # Write target nodes
        f.write(f"TARGETS {num_targets} {' '.join([f'{t[0]},{t[1]}' for t in targets])}\n") # Changed this line to match the expected format in read_input_file

def main():
    os.makedirs("input_files", exist_ok=True)
    sizes = [(5, 5), (10, 10), (15, 15), (20, 20), (25, 25)]  # Different grid sizes
    for i, (rows, cols) in enumerate(sizes):
        generate_input_file(rows, cols, f"input_files/grid_{rows}x{cols}.txt")
        #generate_input_file(rows, cols, f"grid_{rows}x{cols}.txt")

if __name__ == "__main__":
    main()