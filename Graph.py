from queue import PriorityQueue

# Has a unique id, can be uuid or just number them
# Keeps a list of edges that are attached to the vertex
class Vertex:
    def __init__(self, id, location):
        self.id = id
        self.edges = []
        self.location = tuple(location) #tuple of coords

class Dij_Vert:
    def __init__(self, src, w = 1.0):
        self.src = src
        self.weight = w
    def __lt__(self, other):
        return self.weight < other.weight


# Keeps the vertices that it connects in a list
# Has default weight value of 1. The weight is the cost of traversing the edge
class Edge:
    def __init__(self, src, dest, w = 1):
        self.src = src
        self.dest = dest
        self.weight = w

# Keeps a list of vertices, for now, id-ing with numbered list
# Has functions for adding/removing vertices and their edges, dijkstra and A*, and more
class Graph:
    def __init__(self, num_verts):
        self.num_verts = num_verts
        self.graph = [Vertex(i, (0, 0)) for i in range(num_verts)]

    def add_edge(self, src, dest, w = 1.0):
        if (src == dest or not self.is_valid_vertex(src) or not self.is_valid_vertex(dest) or self.has_edge(src, dest)):
            return
        
        edge = Edge(src, dest, w)
        self.graph[src].edges.append(edge)
        self.graph[dest].edges.append(edge)

    # Goes through a vertices edges, if its there, swap the last edge to its place, remove the last edge
    def remove_edge(self, src, dest):
        if (src == dest or not self.is_valid_vertex(src) or not self.is_valid_vertex(dest)):
            return
        
        edges = self.graph[src].edges

        for i, edge in enumerate(edges):
            if edge.dest == dest:
                edges.pop(i)
                break

    
    def compare_dij(self, dij_vert_1, dij_vert_2):
        return dij_vert_1.weight > dij_vert_2.weight

    # Dijkstra's algorithm, not explaining this, look it up
    def Dijkstra(self, s):
        dist = [float('inf')] * self.num_verts
        pred = [-1] * self.num_verts
        known = [False] * self.num_verts
        frontier = PriorityQueue()

        dist[s] = 0
        frontier.put(Dij_Vert(s,0))

        while not frontier.empty():
            next = frontier.get()

            if (known[next.src]):
                continue

            known[next.src] = True

            for edge in self.graph[next.src].edges:
                if not known[edge.dest]:
                    weight = edge.weight
                    if next.weight + weight < dist[edge.dest]:
                        dist[edge.dest] = next.weight + weight
                        pred[edge.dest] = next.src
                        frontier.put(Dij_Vert(edge.dest, dist[edge.dest]))

        return dist
    
    #TODO: Research and Implement
    def A_star(self, s, e):
        # Initialize arrays
        dist = [float('inf')] * self.num_verts  # Distance from source
        pred = [-1] * self.num_verts  # Predecessor array
        known = [False] * self.num_verts  # Known vertices (visited)
        
        frontier = PriorityQueue()  # Priority queue for frontier

        # Initialize start node
        dist[s] = 0
        frontier.put((0, s))  # (priority, node)

        while not frontier.empty():
            # Get the next node with the smallest priority
            current_priority, current_node = frontier.get()

            # If the node is already processed, skip it
            if known[current_node]:
                continue

            known[current_node] = True

            # If we reached the target node, exit early
            if current_node == e:
                break

            # Explore neighbors
            for edge in self.graph[current_node].edges:
                neighbor = edge.dest
                if known[neighbor]:
                    continue

                # g = distance to neighbor
                g = dist[current_node] + edge.weight
                h = self.calculate_heuristic(neighbor, e)  # Heuristic to target
                f = g + h  # Total cost (priority)

                # Update distance if a better path is found
                if g < dist[neighbor]:
                    dist[neighbor] = g
                    pred[neighbor] = current_node
                    frontier.put((f, neighbor))  # Push to frontier with priority f

        return dist, pred  # Return distances and predecessors


    def is_valid_vertex(self, vertex):
        return 0 <= vertex < self.num_verts
    
    def calculate_heuristic(self, start, end):
        start_loc = self.graph[start].location
        end_loc = self.graph[end].location
        return abs(start_loc[0] - end_loc[0]) + abs(start_loc[1] - end_loc[1])
    
    # Checks if there is already an edge between 2 vertices, would need to be changed if the graph is not directed
    def has_edge(self, src, dest):
        return any(edge.dest == dest for edge in self.graph[src].edges)


