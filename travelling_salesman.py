import random
import matplotlib.pyplot as plt
import math
from collections import Counter
import time

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
    
        self.path = []
        self.path_length = 0

    def choose_dest(self, edges_pheromone, dst_power, pheromone_power, activate_pheromones):
        desiderabilities = {}
        total_desiderabilities = 0
        
        # compute desiderability for each vertex
        for dest_vertex in self.visited_vertices_dict.keys():
            if self.visited_vertices_dict[dest_vertex] == False:
                # print(self.curr_vertex)
                # print(dest_vertex)
                dest_vertex_distance = euclidean_distance(self.curr_vertex, dest_vertex)
                temp_edge = get_edge(edges_pheromone, self.curr_vertex, dest_vertex)
                if activate_pheromones == True and temp_edge != None:
                    pheromone_intensity = edges_pheromone[temp_edge]
                    start_time = time.time()
                    desiderability = math.pow((1/dest_vertex_distance), dst_power) * math.pow(pheromone_intensity, pheromone_power)
                    computation_time = time.time() - start_time
                else:
                    start_time = time.time()
                    desiderability = math.pow((1/dest_vertex_distance), dst_power)
                    computation_time = time.time() - start_time
                desiderabilities[desiderability] = dest_vertex
                total_desiderabilities = total_desiderabilities + desiderability
        
        # choose vertex randomly, but by also taking into account the desiderability
        desiderabilities_pdf = [desiderability/total_desiderabilities for desiderability in list(desiderabilities.keys())]
        dest_item = random.choices(population=list(desiderabilities.items()), weights=desiderabilities_pdf, k=1)
        dest_vertex = dest_item[0][1]

        # update ant path
        chosen_edge = Edge(self.curr_vertex, dest_vertex)
        self.path_length = self.path_length + euclidean_distance(self.curr_vertex, dest_vertex)
        self.path.append(chosen_edge)

        '''
        chosen_edge = get_edge(edges_pheromone, self.curr_vertex, dest_vertex)
        if activate_pheromones == True and chosen_edge != None:
            pheromone_released = math.pow(edges_pheromone[chosen_edge], pheromone_power)
        else:
            pheromone_released = 0
        '''
        
        # print("--------")
        # print(dest_vertex)
        
        #flag the choosen vertex as visited and update current position
        self.visited_vertices_dict[dest_vertex] = True
        self.curr_vertex = dest_vertex

        return chosen_edge, computation_time
        # return chosen_edge, pheromone_released

class Ant_colony:
    def __init__(self, ants_n, graph):
        self.ants = []
        self.ants_n = ants_n
        self.graph = graph
        self.create_colony()
    
    def create_colony(self):
        self.ants = []
        for i in range(self.ants_n):
            self.ants.append(Ant(self.graph))

    def get_better_path(self):
        path_lengths = {}
        for ant in self.ants:
            path_lengths[ant]=ant.path_length
        best_ant = min(path_lengths, key=path_lengths.get)
        return best_ant.path, best_ant.path_length

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
        return hash((self.x, self.y))
    
    def get_coordinates(self):
        return (self.x, self.y)
    
    def __str__(self):
        return "x: " + str(self.x) + " - y: " + str(self.y)

class Edge:
    def __init__(self, u, v):
        self.u = u
        self.v = v
        self.compute_distance()
        self.compute_id()

    def __eq__(self, edge):
        if edge != None:
            return (edge.u == self.u and edge.v == self.v) or (edge.u == self.v and edge.v == self.u)
        else:
            return False

    def __hash__(self):
        return hash(self.id)

    def compute_distance(self):
        self.distance = euclidean_distance(self.u, self.v)

    def compute_id(self):
        # self.id = id(self.u.x) + id(self.u.y) + id(self.v.x) + id(self.v.y)
        self.id = self.u.x + self.u.y + self.v.x + self.v.y

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
        self.edges.append(self.find_remaining_edge())
    
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
            
            colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
            color = colors[i]
            i = (i + 1)%len(colors)
            plt.plot([edge.u.x, edge.v.x], [edge.u.y, edge.v.y], color=color)
        
        plt.show()
        return fig


class Ant_opt_alg:

    def __init__(self, iterations, graph, ants_n, dst_power, pheromone_power, evaporation_intensity):
        
        # Alg params
        self.iterations = iterations
        self.locations_num = len(graph.vertices)
        self.graph = graph
        self.ants_n = ants_n
        self.dst_power = dst_power
        self.pheromone_power = pheromone_power
        self.evaporation_intensity = evaporation_intensity
        
        # Support structures
        self.edges_pheromone = {}
        self.colony = Ant_colony(self.ants_n, self.graph)

        self.results = []
        self.paths = []
        self.best_path = []
        self.best_path_length = 9999

        self.execution_time = []

    def compute_step(self, activate_pheromones):
        temp_pheromones = {}
        
        # for each ant, compute the chosen edge and the relative pheromone released 
        #computation_time = 0
        for ant in self.colony.ants:
            # edge, pheromone_released = ant.choose_dest(self.edges_pheromone, self.dst_power, self.pheromone_power, activate_pheromones)
            edge, temp = ant.choose_dest(self.edges_pheromone, self.dst_power, self.pheromone_power, activate_pheromones)
            #computation_time = computation_time + temp
            if edge not in temp_pheromones.keys():
                temp_pheromones[edge] = 1
            else:
                temp_pheromones[edge] = temp_pheromones[edge] + 1

        # update each edge with it's own value but taking into account evaporation
        #computation_time = 0
        #start_time = time.time()
        
        for edge in self.edges_pheromone.keys():
            self.edges_pheromone[edge] =  self.edges_pheromone[edge] * (1 - self.evaporation_intensity)
        computation_time = len(self.edges_pheromone)
        # computation_time = time.time() - start_time

        # for each edge choice, update the relative global pheromone released
        for edge in temp_pheromones.keys():
            if edge in self.edges_pheromone.keys():
                self.edges_pheromone[edge] = self.edges_pheromone[edge] + temp_pheromones[edge]
            else:
                # print("EDGE: " + str(hash(edge)) + " - " + str(edge.u) + " - " + str(edge.v))
                self.edges_pheromone[edge] = temp_pheromones[edge]

        return computation_time

    def exec(self):
        i = 0
        for iteration in range(self.iterations):
            print("iteration: " + str(i))
            i = i + 1

            #computation_time = 0
            for location in range(self.locations_num-1):
                if iteration == 0:
                    temp = self.compute_step(False)
                    #computation_time = computation_time + temp
                else:
                    temp = self.compute_step(True)
                    #computation_time = computation_time + temp
                    
            self.execution_time.append(temp)


            '''
            chosen_edges = Counter(self.edges_pheromone).most_common(self.locations_num-1)
            chosen_edges = [key for key, val in chosen_edges]
            '''

            # chosen_edges = self.get_path(self.edges_pheromone)
            chosen_edges, path_length = self.colony.get_better_path()
            #print("PATH LENGTH: " + str(best_ant.path_length))
            # self.paths.append(chosen_edges)
            self.results.append(path_length)
            
            
            if iteration == self.iterations-1:
                print("LAST GRAPH: " + str(path_length))
                self.graph.set_edges(chosen_edges)
                fig = self.graph.print_graph()
                fig.savefig("Reports"+"/last_path-seed_" + str(alg_seed) + "-nodes_" + 
                str(self.locations_num) + "-iterations_" + str(self.iterations) +
                "-ants_" + str(self.ants_n) + "-" + str(self.dst_power) + "-" +
                str(self.pheromone_power) + "-" + str(self.evaporation_intensity) + ".png")
            

            if path_length < self.best_path_length:
                self.best_path_length = path_length
                self.best_path = chosen_edges

            # reset ants
            self.colony = Ant_colony(self.ants_n, self.graph)
  
        print("BEST LENGTH: " + str(self.best_path_length))
        self.graph.set_edges(self.best_path)
        fig = self.graph.print_graph()
        
        fig.savefig("Reports"+"/best_path-seed_" + str(alg_seed) + "-nodes_" + 
        str(self.locations_num) + "-iterations_" + str(self.iterations) +
        "-ants_" + str(self.ants_n) + "-" + str(self.dst_power) + "-" +
        str(self.pheromone_power) + "-" + str(self.evaporation_intensity) + ".png")
        
        # self.print_paths()
        self.print_results()
        # self.print_time()

    def print_results(self):
        fig, ax = plt.subplots()
        plt.plot(self.results, label="Path length")
        plt.show()
        
        fig.savefig("Reports"+"/results-seed_" + str(alg_seed) + "-nodes_" + 
        str(self.locations_num) + "-iterations_" + str(self.iterations) +
        "-ants_" + str(self.ants_n) + "-" + str(self.dst_power) + "-" +
        str(self.pheromone_power) + "-" + str(self.evaporation_intensity) + ".png")
        
        plt.close()

    def print_paths(self):
        for path in self.paths:
            fig, ax = plt.subplots()
            plt.clf()
        
            for edge in path:
                
                #print(edge.u.get_coordinates(), edge.v.get_coordinates(), color)
                plt.scatter(edge.u.x, edge.u.y, s=10, c='green')
                plt.scatter(edge.v.x, edge.v.y, s=10, c='green')
                plt.plot([edge.u.x, edge.v.x], [edge.u.y, edge.v.y], color='red')
            #print("")
            plt.pause(0.5)

        plt.show()


    def print_time(self):
        fig, ax = plt.subplots()
        plt.plot(self.execution_time, label="Path length")
        plt.show()
        plt.close()
        

graph_seed = 2
alg_seed = 2

if __name__ == '__main__':
    
    random.seed(graph_seed)
    graph = Graph(15, 0, 100, 0, 100)
    graph.create_graph()
    graph.print_graph()
    
    random.seed(alg_seed)
    # iterations, graph, ants_n, dst_power, pheromone_power, evaporation_intensity
    alg = Ant_opt_alg(30, graph, 20, 5, 5, 0.3)
    alg.exec()