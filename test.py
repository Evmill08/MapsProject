from Graph import Graph

# Create a graph and add edges
g = Graph(5)
g.add_edge(0, 1, 2)
g.add_edge(1, 2, 3)
g.add_edge(0, 3, 1)
g.add_edge(3, 4, 5)
g.add_edge(4, 2, 2)

print("Number of vertices:", g.num_verts)
g.show_locations()

# Run A* from source to destination
path, total_cost = g.A_star(0, 2)

if path is not None:
    print("\nPath found:")
    print("Path:", ' -> '.join(str(vertex) for vertex in path))
    print("Total cost:", total_cost)
    
    # Print the coordinates of the path
    print("\nPath coordinates:")
    for vertex in path:
        loc = g.graph[vertex].location
        print(f"Vertex {vertex}: {loc}")
else:
    print("\nNo path found between the vertices")

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