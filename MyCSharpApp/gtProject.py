# Import necessary libraries
import networkx as nx  # For graph creation and manipulation
import matplotlib.pyplot as plt  # For visualizing the graphs
import random  # For generating random weights for edges
import heapq  # For implementing priority queues in Dijkstra's and Prim's algorithms
import os
import time
from memory_profiler import memory_usage
import psutil

# Function to create a city grid graph with random weights and unique node names
def create_city_grid_graph(rows, cols):
    """
    Creates a grid-like city graph with specified rows and columns.
    Each intersection (node) is uniquely named, and roads (edges) have random weights.
    """
    G = nx.grid_2d_graph(rows, cols)  # Creates a 2D grid graph with given dimensions

    # Assign random weights (distance) to each edge
    for (u, v, w) in G.edges(data=True):
        w['weight'] = random.randint(1, 10)  # Random distance between 1 and 10 units

    # Generate unique names for each node for better readability
    location_names = {}
    location_count = 1
    for node in G.nodes():
        location_names[node] = f"Location {location_count}"  # Naming nodes as "Location 1", "Location 2", etc.
        location_count += 1

    # Add these names as attributes to the graph nodes
    nx.set_node_attributes(G, location_names, "name")
    return G, location_names

# Custom implementation of Dijkstra's algorithm
def dijkstra(G, start, target):
    """
    Finds the shortest path between two nodes using Dijkstra's algorithm.
    Returns the total distance and the path as a list of nodes.
    """
    queue = [(0, start, [])]  # Priority queue: (distance, current_node, path_so_far)
    visited = set()  # To track nodes that have already been visited

    while queue:
        dist, current_node, path = heapq.heappop(queue)  # Get the node with the smallest distance

        if current_node in visited:
            continue  # Skip already visited nodes

        # Add the current node to the path and mark it as visited
        path = path + [current_node]
        visited.add(current_node)

        if current_node == target:  # Target reached, return results
            return dist, path

        # Explore neighbors of the current node
        for neighbor, edge_data in G[current_node].items():
            if neighbor not in visited:
                weight = edge_data['weight']
                heapq.heappush(queue, (dist + weight, neighbor, path))

    return float('inf'), []  # Return "infinity" if target is unreachable

# Function to find shortest paths from the starting node to multiple target nodes
def dijkstra_shortest_paths(G, start, targets):
    """
    Uses Dijkstra's algorithm to find the shortest path from the start node to each target node.
    Outputs the paths and their cumulative weights.
    """
    paths = {}
    print("Shortest Paths using Dijkstra's Algorithm:")

    for target in targets:
        total_weight, path = dijkstra(G, start, target)  # Find path to each target
        paths[target] = path
        print(f"\nPath from {start} to {target}:")
        accumulated_weight = 0
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            weight = G[u][v]['weight']
            accumulated_weight += weight
            print(f"Edge: {u} -> {v} (Weight: {weight}), Accumulated Total Weight: {accumulated_weight}")

    return paths

# Custom implementation of Prim's algorithm for MST
def mst_Prim(graph):
    """
    Generates a Minimum Spanning Tree (MST) for the input graph using Prim's algorithm.
    Returns the MST and its total weight.
    """
    mst = nx.Graph()  # Initialize an empty graph to store the MST
    visited = set()  # Track visited nodes
    start_node = list(graph.nodes())[0]  # Arbitrarily choose the first node

    # Priority queue for edges, initialized with edges from the starting node
    edges = [(edge_data['weight'], start_node, v) for v, edge_data in graph[start_node].items()]
    heapq.heapify(edges)
    visited.add(start_node)

    total_weight = 0  # Track the total weight of the MST

    while edges:
        weight, u, v = heapq.heappop(edges)  # Get the smallest edge
        if v not in visited:
            mst.add_edge(u, v, weight=weight)  # Add edge to MST
            total_weight += weight
            visited.add(v)  # Mark the node as visited

            # Add edges from the newly added node to the queue
            for next_v, edge_data in graph[v].items():
                if next_v not in visited:
                    heapq.heappush(edges, (edge_data['weight'], v, next_v))

    return mst, total_weight

# Approximation of a Steiner tree using an auxiliary graph and MST
def approximate_steiner_tree(G, start, targets):
    """
    Approximates a Steiner tree connecting the start node to all target nodes.
    Uses a Minimum Spanning Tree (MST) of an auxiliary graph formed by the shortest paths between required nodes.
    """
    required_nodes = [start] + targets
    auxiliary_graph = nx.Graph()  # Initialize an auxiliary graph

    # Populate auxiliary graph with shortest path distances between required nodes
    for i in range(len(required_nodes)):
        for j in range(i + 1, len(required_nodes)):
            node1, node2 = required_nodes[i], required_nodes[j]
            shortest_path_length, _ = dijkstra(G, node1, node2)
            auxiliary_graph.add_edge(node1, node2, weight=shortest_path_length)

    # Generate MST from auxiliary graph
    mst_auxiliary, _ = mst_Prim(auxiliary_graph)
    steiner_tree = nx.Graph()  # Final Steiner tree
    total_weight = 0  # Total weight of the Steiner tree

    print("\nEdges in the Steiner Tree:")
    for u, v, data in mst_auxiliary.edges(data=True):
        _, path = dijkstra(G, u, v)  # Retrieve actual path in the original graph
        for i in range(len(path) - 1):
            edge_u, edge_v = path[i], path[i + 1]
            weight = G[edge_u][edge_v]['weight']
            if not steiner_tree.has_edge(edge_u, edge_v):  # Avoid duplicate edges
                steiner_tree.add_edge(edge_u, edge_v, weight=weight)
                total_weight += weight
                print(f"Edge: {edge_u} -> {edge_v} (Weight: {weight}), Accumulated Total Weight: {total_weight}")

    return steiner_tree

# Visualization of shortest paths
def visualize_shortest_paths(G, start, shortest_paths, location_names):
    """Visualizes the city graph and shortest paths on a plot with location names."""
    pos = {(x, y): (y, -x) for x, y in G.nodes()}  # positions for a 2D grid
    plt.figure(figsize=(25, 25))

    # Draw the city grid
    nx.draw(G, pos, node_size=50, node_color="lightgray", edge_color="lightgray", with_labels=False, label="City Grid")

    # Draw edge weights
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=16, font_color="blue")

    # Draw shortest paths
    for path in shortest_paths.values():
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="blue", width=2, label="Shortest Paths")

    # Highlight start and target nodes
    nx.draw_networkx_nodes(G, pos, nodelist=[start], node_size=150, node_color="red", label="Start Node")
    targets = list(shortest_paths.keys())
    nx.draw_networkx_nodes(G, pos, nodelist=targets, node_size=150, node_color="orange", label="Target Nodes")

    # Add location names as labels
    location_labels = {node: location_names[node] for node in G.nodes()}
    nx.draw_networkx_labels(G, pos, labels=location_labels, font_size=10)

    # Custom legend to avoid duplicate labels
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc="upper right")
    plt.title("Emergency Response Routes (Shortest Paths)")
    plt.show()

# Visualization of Steiner tree
def visualize_steiner_tree(G, start, targets, steiner_tree, location_names):
    """Visualizes the city graph and Steiner tree on a plot with location names."""
    pos = {(x, y): (y, -x) for x, y in G.nodes()}  # positions for a 2D grid
    plt.figure(figsize=(25, 25))

    # Draw the city grid
    nx.draw(G, pos, node_size=50, node_color="lightgray", edge_color="lightgray", with_labels=False, label="City Grid")

    # Draw edge weights
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=16, font_color="blue")

    # Draw Steiner tree edges
    nx.draw_networkx_edges(G, pos, edgelist=steiner_tree.edges(), edge_color="green", width=2, label="Steiner Tree")

    # Highlight start and target nodes
    nx.draw_networkx_nodes(G, pos, nodelist=[start], node_size=150, node_color="red", label="Start Node")
    nx.draw_networkx_nodes(G, pos, nodelist=targets, node_size=150, node_color="orange", label="Target Nodes")

    # Add location names as labels
    location_labels = {node: location_names[node] for node in G.nodes()}
    nx.draw_networkx_labels(G, pos, labels=location_labels, font_size=10)

    # Custom legend to avoid duplicate labels
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc="upper right")
    plt.title("Emergency Response Routes (Approximate Steiner Tree)")
    plt.show()


# Function to read graph data and parameters from a file
def read_input_file(filepath):
    """Reads a graph and parameters from a file and constructs the city grid graph."""
    with open(filepath, "r") as f:
        lines = f.readlines()

    rows, cols = map(int, lines[0].split())  # Grid dimensions
    G = nx.grid_2d_graph(rows, cols)

    edges = []
    start = None
    targets = []

    for line in lines[1:]:
        line = line.strip()
        if not line:  # Skip empty lines
            continue

        if line.startswith("START"):
            try:
                start_info = line.split()
                if len(start_info) == 2:  # Expecting something like "START (x, y)"
                    coords = start_info[1].strip("()").split(",")
                    if len(coords) == 2:
                        start = tuple(map(int, coords))
                    else:
                        print(f"Skipping invalid START line (wrong format): {line}")
                        continue
                else:
                    print(f"Skipping invalid START line: {line}")
            except ValueError as e:
                print(f"Error processing START line: {e}. Skipping line: {line}")
                continue

        elif line.startswith("TARGETS"):
            try:
                targets_info = line.split()
                if len(targets_info) >= 3:  # Expecting something like "TARGETS 2 (x1, y1) (x2, y2)"
                    num_targets = int(targets_info[1])
                    targets = [
                        tuple(map(int, target.strip("()").split(",")))
                        for target in targets_info[2:]
                    ]
                else:
                    print(f"Skipping invalid TARGETS line (wrong format): {line}")
            except ValueError as e:
                print(f"Error processing TARGETS line: {e}. Skipping line: {line}")
                continue

        else:
            # Process edge lines
            parts = line.split()
            if len(parts) != 3:
                print(f"Skipping invalid edge line: {line}")
                continue  # Skip lines that don't have exactly 3 parts
            try:
                u = tuple(map(int, parts[0].strip("()").split(",")))
                v = tuple(map(int, parts[1].strip("()").split(",")))
                w = int(parts[2])
                edges.append((u, v, w))
            except ValueError as e:
                print(f"Skipping invalid edge line due to error: {e}. Line: {line}")
                continue  # Skip lines with invalid data

    # If no start or targets were found, raise an error
    if not start or not targets:
        raise ValueError("Start or target nodes are missing in the input file.")

    G.clear_edges()  # Clear default grid edges
    G.add_weighted_edges_from(edges)

    location_names = {}
    location_count = 1
    for node in G.nodes():
        location_names[node] = f"Location {location_count}"
        location_count += 1

    nx.set_node_attributes(G, location_names, "name")
    return G, location_names, start, targets





# Main Execution
if __name__ == "__main__":
    # Display available files
    input_dir = "input_files"
    input_files = [f for f in os.listdir(input_dir) if f.endswith(".txt")]
    print("Available input files:")
    for i, filename in enumerate(input_files):
        print(f"{i + 1}. {filename}")

    # Measure memory usage in bytes before execution
    process = psutil.Process(os.getpid())
    mem_before = process.memory_info().rss  # Resident Set Size in bytes

    # User selects a file
    file_index = int(input("Select a file by number: ")) - 1
    selected_file = os.path.join(input_dir, input_files[file_index])

    start_time = time.time()  # Record the start time

    # Read data from the file
    G, location_names, start, targets = read_input_file(selected_file)

    # Display start and target information
    print(f"\nStarting Point: {start}")
    print(f"Target Nodes: {targets}\n")

    # Find shortest paths from start to each target
    shortest_paths = dijkstra_shortest_paths(G, start, targets)

    # Approximate Steiner Tree
    steiner_tree = approximate_steiner_tree(G, start, targets)

    end_time = time.time()  # Record the end time
    print(f"\nExecution Time: {(end_time - start_time) * 1000} ms\n")

    # Measure memory usage in bytes after execution
    mem_after = process.memory_info().rss
    print(f"\nMemory Usage: {mem_after - mem_before} bytes\n")

    # Visualizations
    visualize_shortest_paths(G, start, shortest_paths, location_names)
    visualize_steiner_tree(G, start, targets, steiner_tree, location_names)
