import random
import matplotlib.pyplot as plt
import math
from collections import Counter

def euclidean_distance(u, v):
    distance = math.sqrt(math.pow((v.x - u.x), 2) + math.pow((v.y - u.y), 2))
    return distance

def get_edge(edges, u, v):
    temp_edge = Edge(u, v)
    for edge in edges.keys():
        if temp_edge == edge: 
            return edge
    return None

class Ant:
    def __init__(self, graph, origin=None):

        if origin != None:
            self.curr_vertex = origin
        else:
            self.curr_vertex = random.choice(graph.vertices)

        self.visited_vertices_dict = {}
        for vertex in graph.vertices:
            self.visited_vertices_dict[vertex] = False
        self.visited_vertices_dict[self.curr_vertex] = True
    
    def choose_dest(self, edges_pheromone, dst_power, pheromone_power, activate_pheromones):
        desiderabilities = {}
        total_desiderabilities = 0
        
        #compute desiderability for each vertex
        for dest_vertex in self.visited_vertices_dict.keys():
            if self.visited_vertices_dict[dest_vertex] == False:
                # print(self.curr_vertex)
                # print(dest_vertex)
                dest_vertex_distance = euclidean_distance(self.curr_vertex, dest_vertex)
                temp_edge = get_edge(edges_pheromone, self.curr_vertex, dest_vertex)
                if activate_pheromones == True and temp_edge != None:
                    # temp_edge = get_edge(edges_pheromone, self.curr_vertex, dest_vertex)
                    # if temp_edge != None:
                    pheromone_intensity = edges_pheromone[temp_edge]
                    desiderability = math.pow((1/dest_vertex_distance), dst_power) * math.pow(pheromone_intensity, pheromone_power)
                else:
                    desiderability = math.pow((1/dest_vertex_distance), dst_power)
                desiderabilities[desiderability] = dest_vertex
                total_desiderabilities = total_desiderabilities + desiderability 
        
        #choose vertex randomly, but by also taking into account the desiderability
        desiderabilities_pdf = [desiderability/total_desiderabilities for desiderability in list(desiderabilities.keys())]
        dest_item = random.choices(population=list(desiderabilities.items()), weights=desiderabilities_pdf, k=1)
        dest_vertex = dest_item[0][1]
        chosen_des = dest_item[0][0]
        # print("--------")
        # print(dest_vertex)
        edge = Edge(self.curr_vertex, dest_vertex)
        
        #flag the choosen vertex as visited and update current position
        self.visited_vertices_dict[dest_vertex] = True
        self.curr_vertex = dest_vertex

        return edge, chosen_des

class Vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y
  
    def __eq__(self, vertex):
        if vertex != None:
            return vertex.x == self.x and vertex.y == self.y
        else:
            return False

    def __hash__(self):
        return hash(id(self))
    
    def get_coordinates(self):
        return (self.x, self.y)
    
    def __str__(self):
        return "x: " + str(self.x) + " - y: " + str(self.y)

class Edge:
    def __init__(self, u, v):
        self.u = u
        self.v = v

    def __eq__(self, edge):
        if edge != None:
            return edge.u == self.u and edge.v == self.v
        else:
            return False

    def __hash__(self):
        return hash(id(self))

    def opposite(self, u):
        if u == self.u:
            return self.v
        else:
            return self.u
        return None

class Graph:
    def __init__(self, n, x_min, x_max, y_min, y_max):
        self.n = n
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.vertices = []
        self.edges = []

    def create_graph(self):
        random.seed(1)
        for i in range(self.n):
            x = random.randint(self.x_min, self.x_max)
            y = random.randint(self.y_min, self.y_max)
            vertex = Vertex(x, y)
            self.add_vertex(vertex)

    def add_vertex(self, vertex):
        self.vertices.append(vertex)

    def add_edge(self, edge):
        self.edges.append(edge)
    
    def set_edges(self, edges):
        self.edges = []
        self.edges = edges
        # self.edges.append(self.find_remaining_edge())
    
    def find_remaining_edge(self):
        vertex_edges_counter = {}
        for edge in self.edges:
            if edge.u not in vertex_edges_counter: 
                vertex_edges_counter[edge.u] = 1
            else:
                vertex_edges_counter[edge.u] = vertex_edges_counter[edge.u] + 1
            if edge.v not in vertex_edges_counter:
                vertex_edges_counter[edge.v] = + 1
            else:
                vertex_edges_counter[edge.v] = vertex_edges_counter[edge.v] + 1

        remaining_vertices = []
        for vertex in vertex_edges_counter.keys():
            if vertex_edges_counter[vertex] == 1:
                remaining_vertices.append(vertex)
        
        return Edge(remaining_vertices[0], remaining_vertices[1])

    def print_graph(self):

        fig, ax = plt.subplots()
    
        for vertex in self.vertices:
            # print(vertex.x, vertex.y)
            plt.scatter(vertex.x, vertex.y, s=10, c='green')

        i = 0
        for edge in self.edges:
            '''
            colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
            color = colors[i]
            i = (i + 1)%len(colors)
            print(edge.u.get_coordinates() , edge.v.get_coordinates(), color)
            '''
            plt.plot([edge.u.x, edge.v.x], [edge.u.y, edge.v.y], color='red')

        plt.show()

class Ant_opt_alg:

    def __init__(self, iterations, graph, ants_n, dst_power, pheromone_power, evaporation_intensity):
        self.iterations = iterations
        self.locations_num = len(graph.vertices)
        self.graph = graph
        self.ants_n = ants_n
        self.dst_power = dst_power
        self.pheromone_power = pheromone_power
        self.evaporation_intensity = evaporation_intensity
        self.edges_pheromone = {}
        self.create_ants(self.ants_n)

        self.pheromone_matrix = {}
        
    def create_ants(self, ants_n):
        self.ants = []
        for i in range(ants_n):
            self.ants.append(Ant(self.graph))

    def compute_step(self, activate_pheromones):
        temp_desiderabilites = {}
        
        # for each ant, compute the chosen edge and the relative desiderability 
        for ant in self.ants:
            edge, desiderability = ant.choose_dest(self.edges_pheromone, self.dst_power, self.pheromone_power, activate_pheromones)
            temp_desiderabilites[edge] = desiderability
        
        # update each edge with it's own value but taking into account evaporation
        for edge in self.edges_pheromone.keys():
            self.edges_pheromone[edge] =  (self.edges_pheromone[edge] * (1 - self.evaporation_intensity))

        # for each edge choice, update the relative global desiderability
        for edge in temp_desiderabilites.keys():
            if edge in self.edges_pheromone:
                self.edges_pheromone[edge] = self.edges_pheromone[edge] + temp_desiderabilities[edge]
            else:
                self.edges_pheromone[edge] = temp_desiderabilites[edge]

    def exec(self):
        for iteration in range(self.iterations):
            for location in range(self.locations_num-1):
                if iteration == 0:
                    self.compute_step(False)
                else:
                    self.compute_step(True)

            '''
            chosen_edges = Counter(self.edges_pheromone).most_common(self.locations_num-1)
            chosen_edges = [key for key, val in chosen_edges]
            '''

            chosen_edges = self.get_path(self.edges_pheromone)
            self.graph.set_edges(chosen_edges)
            self.graph.print_graph()

            self.create_ants(self.ants_n)


    ## Auxiliary function
    def edges_to_matrix(self, edges):
        print(len(edges))
        pheromone_matrix = {}
        for edge, pheromone in edges.items():
            if edge.u not in pheromone_matrix:
                pheromone_matrix[edge.u] = {}
                pheromone_matrix[edge.u][edge.v] = pheromone
            else:
                pheromone_matrix[edge.u][edge.v] = pheromone
        return pheromone_matrix
        
    ## Auxiliary function
    def get_path(self, edges):
        pheromone_matrix = self.edges_to_matrix(edges)
        self.print_edges_matrix(pheromone_matrix)
        path = []
        visited = []
        for u, edges in pheromone_matrix.items():
            done = False
            temp_edges = edges.copy()
            while done != True:
                try:
                    v = max(temp_edges, key=temp_edges.get)  
                    if v not in visited:
                        visited.append(v)
                        path.append(Edge(u, v))
                        done = True
                    else:
                        temp_edges.pop(v)
                except ValueError:
                    done = True
        return path

    ## Auxiliary function
    def print_edges_matrix(self, matrix):
        i = 0
        for u, edges in matrix.items():
            pheromone_values = ""
            for v, pheromone in edges.items():
                print("temp")
                pheromone_values = pheromone_values + "\t" + str(pheromone)
            print(str(i) + ":\t" + pheromone_values)
            i = i + 1
        print("\n")


if __name__ == '__main__':
    graph = Graph(10, 0, 100, 0, 100)
    graph.create_graph()
    graph.print_graph()
    # iterations, graph, ants_n, dst_power, pheromone_power, evaporation_intensity
    alg = Ant_opt_alg(10, graph, 1, 1, 1, 0.5)
    alg.exec()