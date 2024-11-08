import numpy as np
import heapq
#import matplotlib.pyplot as plt
#from matplotlib.patches import FancyArrowPatch


class Pathfinder(object):
    def __init__(self, position = 0, orientation = 1) -> None:
        self.pos = position
        self.prev = position
        self.orientation = orientation
        self.prev_orientation = orientation
        self.mat = np.zeros((49, 49))
        self.obj = 0
        self.path = iter([self.pos])
        self.directions = {
            0: (0, 1),    # Nord
            1: (1, 0),    # Est
            2: (0, -1),   # Sud
            3: (-1, 0)    # Ouest
        }

        for i in range(49):
            
            if i % 7 != 6:  
                self.mat[i, i + 1] = 1
                self.mat[i + 1, i] = 1
            if i % 7 != 0:  
                self.mat[i, i - 1] = 1
                self.mat[i - 1, i] = 1
            if i >= 7:
                self.mat[i, i - 7] = 1
                self.mat[i - 7, i] = 1
            if i < 20:  
                self.mat[i, i + 7] = 1
                self.mat[i + 7, i] = 1


    def restart_mat(self):
        self.mat = np.zeros((49, 49))

        for i in range(49):
            
            if i % 7 != 6:  
                self.mat[i, i + 1] = 1
            if i % 7 != 0:  
                self.mat[i, i - 1] = 1
            if i >= 7:
                self.mat[i, i - 7] = 1
            if i < 20:  
                self.mat[i, i + 7] = 1


    def enleve(self, ind1, ind2):
        self.mat[ind1, ind2] = 0
        self.mat[ind2, ind1] = 0

    def objectif(self, objectif):
        self.obj = objectif

    def __get_direction__(self, start, end):
        if abs(start - end) == 1:
            return 'horizontal'
        elif abs(start - end) == 7:
            return 'vertical'
        return None
    
    def djikstra(self, start, end):
        if start == end:
            return 'arrived'
        n = self.mat.shape[0]
        distances = np.full(n, np.inf)
        distances[self.pos] = 0  # Distance de départ à lui-même est 0
        previous_nodes = np.full(n, -1)  # Tableau pour garder la trace des chemins
        visited = np.zeros(n, dtype=bool)  # Tableau pour garder trace des nœuds visités
        priority_queue = [(0, self.pos, None)]  # File de priorité (distance, nœud)

        while priority_queue:
            current_distance, current_node, last_direction = heapq.heappop(priority_queue)
            if current_node == self.obj:
                break
            if visited[current_node]:
                continue
            visited[current_node] = True
            for neighbor in range(n):
                edge_weight = self.mat[current_node, neighbor]
                if edge_weight > 0 and not visited[neighbor]:
                    new_direction = self.__get_direction__(current_node, neighbor)
                    rotation_penalty = 0 if new_direction == last_direction else 0.7
                    new_distance = current_distance + edge_weight + rotation_penalty
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        previous_nodes[neighbor] = current_node
                        heapq.heappush(priority_queue, (new_distance, neighbor, new_direction))
        path = []
        current = end
        while current != -1:
            path.append(int(current))
            current = previous_nodes[current]
        path = path[::-1]
        if distances[end] == np.inf:
            print('impossible')
            exit()
        self.path = iter(path[1:])

    def decision(self, objectif = -1):
        """
        return -1 if impossible
        return -2 if objectif reached
        else : return angle of rotation necessary to reach the objective
        
        """
        if self.pos == objectif:
            return 'arrived'
        if objectif != self.obj:
            self.obj = objectif
            self.djikstra(self.pos, objectif)
            
        position_suivante = next(self.path)
        self.prev_orientation = self.orientation
        if (position_suivante - self.pos) == 1: 
            self.orientation = 1
        elif (position_suivante - self.pos) == -1:
            self.orientation = 3
        elif (position_suivante - self.pos) == 7:
            self.orientation = 2
        elif (position_suivante - self.pos) == -7:
            self.orientation = 0
        self.prev = self.pos
        self.pos = position_suivante
        print('prev:', self.prev_orientation, 'ori:', self.orientation, 'next pos:', self.pos)

        def conv(deg):
            if deg > 180:
                deg = -(deg - 180)

            if deg == 180 or deg == -180:
                return 190
            return deg

        return conv(90 * (self.orientation - self.prev_orientation))

    def enleve_arrete_en_face(self):
        print('enleve:', self.pos, self.prev)
        self.enleve(self.pos, self.prev)
        self.pos = self.prev
        self.djikstra(self.pos, self.obj)
        
    
    def route_barree(self):
        self.orientation = (self.orientation + 2) %2 #on se retourne
        self.enleve(self.pos, self.prev) 
        self.pos = self.prev    
    