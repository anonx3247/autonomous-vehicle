import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch


class Pathfinder(object):
    def __init__(self, position = 0, orientation = 1) -> None:
        self.pos = position
        self.prev = position
        self.orientation = orientation
        self.prev_orientation = orientation
        self.mat = np.zeros((25, 25))
        self.obj = 0
        self.directions = {
            0: (0, 1),    # Nord
            1: (1, 0),    # Est
            2: (0, -1),   # Sud
            3: (-1, 0)    # Ouest
        }

        for i in range(25):
            
            if i % 5 != 4:  
                self.mat[i, i + 1] = 1
                self.mat[i + 1, i] = 1
            if i % 5 != 0:  
                self.mat[i, i - 1] = 1
                self.mat[i - 1, i] = 1
            if i >= 5:
                self.mat[i, i - 5] = 1
                self.mat[i - 5, i] = 1
            if i < 20:  
                self.mat[i, i + 5] = 1
                self.mat[i + 5, i] = 1


    def restart_mat(self):
        self.mat = np.zeros((25, 25))

        for i in range(25):
            
            if i % 5 != 4:  
                self.mat[i, i + 1] = 1
            if i % 5 != 0:  
                self.mat[i, i - 1] = 1
            if i >= 5:
                self.mat[i, i - 5] = 1
            if i < 20:  
                self.mat[i, i + 5] = 1


    def enleve(self, ind1, ind2):
        self.mat[ind1, ind2] = 0
        self.mat[ind2, ind1] = 0

    def objectif(self, objectif):
        self.obj = objectif

    def decision(self, objectif = -1):
        """
        return -1 if impossible
        return -2 if objectif reached
        else : return angle of rotation necessary to reach the objective
        
        """
        if objectif != -1:
            self.obj = objectif
        if self.pos == self.obj:
            return -2 
        n = self.mat.shape[0]
        distances = np.full(n, np.inf)
        distances[self.pos] = 0  # Distance de départ à lui-même est 0
        previous_nodes = np.full(n, -1)  # Tableau pour garder la trace des chemins
        visited = np.zeros(n, dtype=bool)  # Tableau pour garder trace des nœuds visités
        priority_queue = [(0, self.pos)]  # File de priorité (distance, nœud)

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
            if current_node == self.obj:
                break
            if visited[current_node]:
                continue
            visited[current_node] = True
            for neighbor in range(n):
                edge_weight = self.mat[current_node, neighbor]
                if edge_weight > 0 and not visited[neighbor]:
                    new_distance = current_distance + edge_weight
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        previous_nodes[neighbor] = current_node
                        heapq.heappush(priority_queue, (new_distance, neighbor))
        path = []
        current = self.obj
        while current != -1:
            path.append(int(current))
            current = previous_nodes[current]
        path = path[::-1]
        if distances[self.obj] == np.inf:
            return -1
        
        position_suivante = path[1]
        self.prev_orientation = self.orientation
        if (position_suivante - self.pos) == 1: 
            self.orientation = 1
        elif (position_suivante - self.pos) == -1:
            self.orientation = 3
        elif (position_suivante - self.pos) == 5:
            self.orientation = 2
        elif (position_suivante - self.pos) == -5:
            self.orientation = 0
        self.pos = position_suivante
        return ((90 * (self.orientation - self.prev_orientation) + 180 ) % 360 - 180)

    def aucune_sortie(self):
        self.enleve(self.pos, self.prev)

        return self.decision()
    
    def route_barree(self):
        self.orientation = (self.orientation + 2) %2 #on se retourne
        self.enleve(self.pos, self.prev) 
        self.pos = self.prev
        return self.decision()
    
    def grid(self):
        # Fonction pour dessiner la grille avec la position et orientation du robot
        fig, ax = plt.subplots(figsize=(6, 6))

        # Dessiner les nœuds de la grille
        for i in range(5):
            for j in range(5):
                ax.plot(j, 5 - i - 1, 'o', markersize=10, color='gray')

        # Dessiner toutes les arêtes existantes en noir
        for i in range(5):
            for j in range(5):
                current_index = i * 5 + j
                for neighbor in range(25):
                    if self.mat[current_index, neighbor] > 0:  # S'il y a une arête
                        neighbor_i = neighbor // 5
                        neighbor_j = neighbor % 5

                        ax.add_patch(FancyArrowPatch(
                            (j, 5 - i - 1), (neighbor_j, 5 - neighbor_i - 1),
                            arrowstyle='-|>', color='black', mutation_scale=15,
                            linewidth=1, alpha=0.5))

        # Calculer les coordonnées du robot en fonction de la position et de la direction
        robot_x = self.pos % 5
        robot_y = 5 - (self.pos // 5) - 1

        # Calculer la fin de la flèche pour indiquer l'orientation
        dx, dy = self.directions[self.orientation]
        end_x = robot_x + dx * 0.3
        end_y = robot_y + dy * 0.3

        # Dessiner la flèche rouge pour le robot
        ax.add_patch(FancyArrowPatch(
            (robot_x, robot_y), (end_x, end_y),
            arrowstyle='-|>', color='red', mutation_scale=20,
            linewidth=2))

        # Configuration de la visualisation
        ax.set_xlim(-0.5, 5 - 0.5)
        ax.set_ylim(-0.5, 5 - 0.5)
        ax.set_xticks(np.arange(5))
        ax.set_yticks(np.arange(5))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.grid(False)

        plt.title("Grille 5x5 avec position et orientation du robot")
        plt.gca().set_aspect('equal', adjustable='box')

        # Enregistrer l'image dans un fichier
        plt.savefig("grille_5x5_chemin.png", bbox_inches='tight')
        plt.close()  # Fermer la figure pour éviter de l'afficher

    
