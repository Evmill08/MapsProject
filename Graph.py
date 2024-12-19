from queue import PriorityQueue

# Has a unique id, can be uuid or just number them
# Keeps a list of edges that are attached to the vertex
class Vertex:
    def __init__(id, location):
        self.id = id
        self.edges = list(Edge)
        self.location = tuple(location.first, location.second) #tuple of coords

class Dij_Vert:
    def __init__(self, src, w = 1.0):
        self.src = src
        self.weight = w


# Keeps the vertices that it connects in a list
# Has default weight value of 1. The weight is the cost of traversing the edge
class Edge:
    def __init__(src, dest, w = 1):
        self.src = src
        self.dest = dest
        self.weight = w

# Keeps a list of vertices, for now, id-ing with numbered list
# Has functions for adding/removing vertices and their edges, dijkstra and A*, and more
class Graph:
    def __init__(self, num_verts):
        self.num_verts = num_verts
        self.graph = list(Vertex)

        for i in range(num_verts):
            self.graph.append(i)


    def add_edge(self, src, dest, w = 1.0):
        if (src == dest or not self.is_valid_vertex(src) or not self.is_valid_vertex(dest) or self.has_edge(src, dest)):
            return
        
        self.graph[src].edges.append(src, dest, w)
        self.graph[dest].edges.append(dest, src, w)

# Goes through a vertices edges, if its there, swap the last edge to its place, remove the last edge
    def remove_edge(self, src, dest):
        if (src == dest or not self.is_valid_vertex(src) or not self.is_valid_vertex(dest)):
            return
        
        edges = self.graph[src].edges

        for i in range(edges.count()):
            if (edges[i].dest == dest):
                edges[i] = edges[edges.count() - 1]
                edges.pop()
                break

    
    def compare_dij(self, dij_vert_1, dij_vert_2):
        return dij_vert_1.weight > dij_vert_2.weight

    # Dijkstra's algorithm, not explaining this, look it up
    def Dijkstra(self, s):
        dist = [float('inf')] * self.graph.count()
        pred = [-1] * self.graph.count()
        known = [False] * self.graph.count()

        frontier = PriorityQueue()

        dist[s] = 0
        pred[s] = s

        start = Dij_Vert(s, 0)
        frontier.put(start)

        while (not frontier.empty()):
            next = Dij_Vert()
            next.src, next.weight = frontier.get()

            if (known[next.src]):
                continue

            known[next.src] = True

            for edge in self.graph[next.src]:
                if (not known[edge.dest]):
                    weight = edge.weight

                    if (next.weight + weight < dist[edge.src]):
                        dist[edge.dest] = next.weight + weight
                        pred[edge.dest] = next.src
                        frontier.put({edge.dest, next.weight + weight})

        return dist
    
    #TODO: Research and Implement
    def A_star(self, s, e):
        dist = [float('inf')] * self.graph.count()
        pred = [-1] * self.graph.count()
        known = [False] * self.graph.count()
        h_vals = [-1] * self.graph.count()

        frontier = PriorityQueue()

        dist[s] = 0
        pred[s] = s
        h_vals[s] = self.calculate_heuristic(s, e)

        start = Dij_Vert(s, 0)
        frontier.put(start)

        while(not frontier.empty()):
            next = Dij_Vert()
            next.src, next.weight = frontier.get()

            if (known[next.src]):
                continue

            known[next.src] = True

            for edge in self.graph[next.src]:
                if (not known[edge.dest]):
                    weight = edge.weight
                    h = self.calculate_heuristic(next.src, e)

                    if (next.weight + weight + h < dist[edge.src] + self.calculate_heuristic(edge.src, e)):
                        dist[edge.dest] = next.weight + weight
                        pred[edge.dest] = next.src
                        h_vals[edge.dest] = h
                        frontier.put({edge.dest, next.weight + weight + h})

        return dist


    def is_valid_vertex(self, vertex):
        return vertex >= 0 and vertex < self.graph.count()
    
    def calculate_heuristic(self, start, end):
        h = abs(start.location.first - end.location.first) + abs(start.location.second - end.location.second)
        return h
    
    # Checks if there is already an edge between 2 vertices, would need to be changed if the graph is not directed
    def has_edge(self, src, dest):

        if (src == dest or not self.is_valid_vertex(src) or not self.is_valid_vertex(dest)):
            return False

        for edge in self.graph[src].edges:
            if edge.dest == dest:
                return True
            
        return False


