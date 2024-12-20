from Graph import Graph

# Create a graph and add edges
g = Graph(5)
g.add_edge(0, 1, 2)
g.add_edge(1, 2, 3)
g.add_edge(0, 3, 1)
g.add_edge(3, 4, 5)
g.add_edge(4, 2, 2)

# Run A* from source to destination
distances, predecessors = g.A_star(0, 2)
print("Shortest distances:", distances)
print("Predecessors:", predecessors)

# Initialize a graph with 5 vertices
g2 = Graph(5)

# Add edges
g2.add_edge(0, 1, 2)
g2.add_edge(1, 2, 3)
g2.add_edge(0, 3, 1)
g2.add_edge(3, 4, 5)
g2.add_edge(4, 2, 2)

# Run Dijkstra's algorithm from vertex 0
distances = g2.Dijkstra(0)
print("Shortest distances from vertex 0:", distances)