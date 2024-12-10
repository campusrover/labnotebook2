# Graph-Based Navigation for Autonomous Navigation

## Overview

Graph derived navigation is a method where the environment is represented as a **graph**, consisting of **nodes** (representing locations) and **edges** (representing not necessarily paths but instead the interconnectivity between the nodes). 
This method is ideal for scenarios where the robot needs to navigate from one point to another while not necessarily following a pre-defined path.

In this example, the robot uses **Breadth-First Search (BFS)** to find the shortest path between two points on a map. This pathfinding technique is particularly suitable for unweighted graphs, like those used in basic navigation tasks.

---

## Breadth First Search (BSF) Algorithm 

### How does this work? 

**BFS** is a graph traversal algorithm that explores nodes layer by layer, starting from a starting node. BFS is commonly used for pathfinding in graphs because it guarantees finding the shortest path in an unweighted graph, i.e., all edges have no weight. BFS looks at all the neighbors of a node before moving to the next level of nodes. This is ideal for pathfinding between two nodes because it systematically expands outward from the start.

### How BFS Works

1. **Initialization**: BFS starts by visiting the starting node and marking it as visited. This node is then added to the queue.
2. **Exploring Closest Nodes Outward**: The algorithm dequeues a node and explores all of its unvisited nearby nodes. Each unvisited node is added to the queue and marked as visited.
3. **Path Creation**: BFS tracks the path taken to each node. Once the target node is eventually reached, the algorithm reconstructs the path by backtracking thanks to the queue. 
4. **Termination**: The algorithm terminates when it finds the destination node once more. We are guaranteed to find the shortest path, because it explores nodes in order of how many epochs it takes to reach them. 

### Basic Implementation

```python
from collections import deque

class Graph:
    def __init__(self):
        self.nodes = []  # List of nodes (lat, lon)
        self.edges = {}  # Dictionary of edges, mapping the node to list of connected nodes
        # it is also convenient to make a separate dict with the node information, with keys containing easy to remember names. 

    def add_node(self, lat, lon):
        """Add a new node to the graph."""
        self.nodes.append((lat, lon))
        self.edges[(lat, lon)] = []

    def add_edge(self, node1, node2):
        """Add an undirected edge between two nodes. This can be modified to have directions as well."""
        if node1 in self.edges and node2 in self.edges:
            self.edges[node1].append(node2)
            self.edges[node2].append(node1)
        else:
            print("One of the nodes does not exist in the graph!")

    def bfs(self, start_node, end_node):
        """Find the shortest path using BFS from start_node to end_node."""
        queue = deque([(start_node, [start_node])])  # Store nodes with the path taken
        visited = set()  # Set to track visited nodes

        while queue:
            current_node, path = queue.popleft()

            # If we reach the target node, return the path
            if current_node == end_node:
                return path

            visited.add(current_node)  # Mark the node as visited

            # Explore the neighbors (connected nodes)
            for close_alt in self.edges.get(current_node, []):
                if close_alt not in visited:
                    queue.append((close_alt, path + [close_alt]))  # Add node with new path

