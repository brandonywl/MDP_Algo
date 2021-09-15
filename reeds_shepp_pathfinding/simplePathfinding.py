import math
import heapq

from map_tiles.point import Point
from utilities import drawing
from movement_models import collision_detection

#pathfinding with no turning, all nodes have theta equal to the start, except the end
#this is run when reeds-shepp path has an obstacle
class simplePathfinding:
    def __init__(self, start, end, obstacles):
        self.start = start
        self.end = end
        self.obstacles = obstacles

        self.min_x = 0
        self.max_x = 800
        self.min_y = 0
        self.max_y = 800

        self.step = 20
        self.options = [-self.step, 0, self.step]
        self.cost = 1

        self.vehicle_length = drawing.CAR_LENGTH

    def dist(self, source, target):
        diff_x = target[0] - source[0]
        diff_y = target[1] - source[1]
        return math.sqrt(diff_x ** 2 + diff_y ** 2)

    def run(self):
        """
        Each node is represented as [x, y, theta], where theta is in degrees
        """

        """
        Elements in open_heap have the following format:
        (totalCost, node)
        """
        open_heap = [] 

        """
        Elements in open_dict have the following format:
        node: (totalCost, traversedCost, parent)
        """
        open_dict = {} 

        """
        Elements in visited_dict have the following format:
        node: (totalCost, traversedCost, parent)
        """
        visited_dict = {}

        # Start from self.start and end at self.end
        start = self.start
        end = self.end
        distanceToGoal = self.dist(start, end)

        heapq.heappush(open_heap, (distanceToGoal, distanceToGoal, start))
        while open_heap:
            current = open_heap[0]
            currentNode = current[1]
            currentDictItem = open_dict[currentNode]
            visited_dict[currentNode] = (current[0], currentDictItem[1], currentDictItem[2])

            if self.dist(currentNode, end) <= 10:
                print("Simple path found!")
                finalPath = []
                finalPath.append(end)
                finalPath.append(currentNode)
                currentNode = currentDictItem[2]
                while currentNode != start:
                    finalPath.append(currentNode)
                    currentNode = visited_dict[currentNode][2]
                finalPath.append(start)
                return finalPath
            
            heapq.heappop(open_heap)

            for i in self.options:
                x = i + currentNode[0]
                if x < self.min_x or x > self.max_x:
                    continue
                for j in self.options:
                    y = j + currentNode[1]
                    if y < self.min_y or y > self.max_y:
                        continue
                    neighbourNode = [x, y, currentNode[2]]
                    if neighbourNode in visited_dict or collision_detection.has_collision(Point(x, y, currentNode[2]), self.obstacles):
                        continue
                    traversedCost = currentDictItem[1] + self.cost
                    totalCost = self.dist(neighbourNode, end) + traversedCost
                    if neighbourNode in open_dict:
                        if open_dict[neighbourNode][0] > totalCost:
                            open_dict[neighbourNode] = [totalCost, traversedCost, currentNode]
                            continue
                    heapq.heappush(totalCost, neighbourNode)
                    open_dict[neighbourNode] = [totalCost, traversedCost, currentNode]

             
            


