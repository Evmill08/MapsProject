from queue import PriorityQueue
import random

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
        
        for vertex in self.graph:
            self.set_vertex_location(vertex)

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


    def set_vertex_location(self, vertex):
        x = round(random.random() * 100, 4)
        y = round(random.random() * 100, 4)
        vertex.location = (x,y)


    # For Testing
    def show_locations(self):
        for vert in self.graph:
            print(f"Vert {vert.id} Location: ", vert.location)

        
    def compare_dij(self, dij_vert_1, dij_vert_2):
        return dij_vert_1.weight > dij_vert_2.weight

    # Dijkstra's algorithm, not explaining this, look it up
    def Dijkstra(self, s):
        dist = [float('inf')] * self.num_verts
        pred = [-1] * self.num_verts
        known = [False] * self.num_verts
        frontier = PriorityQueue()

        dist[s] = 0
        pred[s] = s
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
        

    def A_star(self, start, end):
        g_scores = {v: float('inf') for v in range(self.num_verts)}
        f_scores = {v: float('inf') for v in range(self.num_verts)}
        open_set = PriorityQueue()
        came_from = {}

        g_scores[start] = 0
        f_scores[start] = self.calculate_heuristic(start, end)

        open_set.put((f_scores[start], start))

        in_open_set = {start}

        while not open_set.empty():
            current = open_set.get()[1]

            if (current == end):
                path = self.reconstruct_path(came_from, end)
                return path, g_scores[end]
            
            in_open_set.remove(current)

            for edge in self.graph[current].edges:
                neighbor = edge.dest

                tent_g_score = g_scores[current] + edge.weight

                if tent_g_score < g_scores[neighbor]:
                    came_from[neighbor] = current
                    g_scores[neighbor] = tent_g_score
                    f_scores[neighbor] = self.calculate_heuristic(neighbor, end) + tent_g_score

                    if (neighbor not in in_open_set):
                        open_set.put((f_scores[neighbor], neighbor))
                        in_open_set.add(neighbor)

        return None, float('inf')




    def is_valid_vertex(self, vertex):
        return 0 <= vertex < self.num_verts
    
    def calculate_heuristic(self, start, end):
        start_loc = self.graph[start].location
        end_loc = self.graph[end].location
        return abs(start_loc[0] - end_loc[0]) + abs(start_loc[1] - end_loc[1])
    
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    # Checks if there is already an edge between 2 vertices, would need to be changed if the graph is not directed
    def has_edge(self, src, dest):
        return any(edge.dest == dest for edge in self.graph[src].edges)


