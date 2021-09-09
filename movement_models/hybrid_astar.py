import math
import heapq
import matplotlib.pyplot as plt

from map_tiles.point import Point
from movement_models import bicycle_movement_model
from utilities import drawing


class Hybrid_AStar:

    def __init__(self, start, end):
        self.vehicle_length = drawing.CAR_LENGTH
        print(self.vehicle_length)

        self.start = start
        self.end = end

        self.min_x = 0
        self.max_x = 800
        self.min_y = 0
        self.max_y = 800

        self.steering_angles = [-30, 0, 30]
        self.add_steering_costs = [0.1, 0, 0.1]

        self.velocity = [-10, 10]
        self.add_velocity_costs = [1, 0]

    def euc_dist(self, position, target):
        diff_x = position[0] - target[0]
        diff_y = position[1] - target[1]
        diff_theta = position[2] - target[2]
        output = math.sqrt(diff_x ** 2 + diff_y ** 2 + math.radians(diff_theta) ** 2)

        return output

    def round(self, x, prec=2, base=1):
        return round(base * round(float(x) / base), prec)

    def run(self):
        """
        Each node is represented as [x, y, theta], where theta is in degrees
        """

        """
        Elements in open_heap have the following format:
        (cost, node_discrete)
        """
        open_heap = [] 

        """
        Elements in open_dict have the following format:
        node_discrete: (cost, node_continuous, (parent_discrete, parent_continuous), (velocity, steering_angle))
        """
        open_dict = {} 

        """
        Elements in visited_dict have the following format:
        node_discrete: (cost, node_continuous, (parent_discrete, parent_continuous), (velocity, steering angle))
        """
        visited_dict = {}

        # We are starting from self.start and ending at self.end
        start = self.start
        end = self.end
        cost_to_neighbour_from_start = 0
        total_cost = cost_to_neighbour_from_start + self.euc_dist(start, end)

        heapq.heappush(open_heap, (total_cost, start))
        open_dict[start] = (total_cost, start, (start, start), (0, 0))

        count = 0
        while open_heap:
            count += 1

            """
            Get the first node in open_heap, i.e. the node that has the lowest 
            total cost
            """
            chosen_d_node = open_heap[0][1]
            chosen_node_total_cost = open_heap[0][0]
            chosen_c_node = open_dict[chosen_d_node][1]

            visited_dict[chosen_d_node] = open_dict[chosen_d_node]

            # diff = self.end[2] - chosen_c_node[2]
            # diff = (diff + 180) % 360 - 180
            # diff = 0

            if self.euc_dist(chosen_d_node, end) < 20:
                print("We are done???")
                print("count: ", count)
                # print("visited_dict: ", visited_dict)
                print("Current continuous", chosen_c_node)
                print()

                reversed_final_path = [(chosen_d_node, chosen_c_node)]
                reversed_states = [open_dict[chosen_d_node][3]]
                temp = chosen_d_node
                print("discrete: ", temp, "state it took: ", open_dict[chosen_d_node][3])
                while True:
                    parent_discrete = visited_dict[temp][2][0]
                    print("discrete: ", parent_discrete, "state it took: ", open_dict[parent_discrete][3])
                    # print(parent_discrete)
                    if parent_discrete == start:
                        break

                    parent_continuous = visited_dict[parent_discrete][1]
                    reversed_final_path.append((parent_discrete, parent_continuous))

                    state = visited_dict[parent_discrete][3]
                    reversed_states.append(state)

                    temp = parent_discrete

                return reversed_final_path[::-1], reversed_states[::-1]

            heapq.heappop(open_heap)

            for i in range(0, len(self.steering_angles)):
                for j in range(0, len(self.velocity)):
                    steering_angle = self.steering_angles[i]
                    velocity = self.velocity[j]

                    cost_to_neighbour_from_start = chosen_node_total_cost - self.euc_dist(chosen_d_node, end) 

                    # continuous coordinates
                    neighbour_x_cts = chosen_c_node[0] + (velocity * math.cos(math.radians(chosen_c_node[2])))
                    neighbour_y_cts = chosen_c_node[1] - (velocity * math.sin(math.radians(chosen_c_node[2])))
                    neighbour_theta_cts = math.radians(chosen_c_node[2]) + \
                                             (velocity * math.tan(math.radians(steering_angle)) / (float(self.vehicle_length)))
                    neighbour_theta_cts = math.degrees(neighbour_theta_cts)

                    # axel_point = Point(chosen_c_node[0], chosen_c_node[1], chosen_c_node[2])
                    # next_point = bicycle_movement_model.move(axel_point, steering_angle, velocity, self.vehicle_length, 90, 60)
                    # neighbour_x_cts, neighbour_y_cts, neighbour_theta_cts = next_point.as_list(False)
                    # print(next_point.as_list(False))
                    # discrete coordinates
                    neighbour_x_d = self.round(neighbour_x_cts)
                    neighbour_y_d = self.round(neighbour_y_cts)
                    neighbour_theta_d = self.round(neighbour_theta_cts)

                    neighbour = ((neighbour_x_d, neighbour_y_d, neighbour_theta_d), 
                                    (neighbour_x_cts, neighbour_y_cts, neighbour_theta_cts))

                    if neighbour_x_d >= self.min_x and neighbour_x_d < self.max_x \
                        and neighbour_y_d >= self.min_y and neighbour_y_d < self.max_y:

                        heuristic_cost = self.euc_dist((neighbour_x_d, neighbour_y_d, neighbour_theta_d), end)
                        cost_to_neighbour_from_start += abs(velocity) + self.add_steering_costs[i] + self.add_velocity_costs[j]
                        total_cost = heuristic_cost + cost_to_neighbour_from_start

                        skip = 0

                        if neighbour[0] in open_dict:
                            if total_cost > open_dict[neighbour[0]][0]:
                                skip = 1
                        elif neighbour[0] in visited_dict:
                            if total_cost > visited_dict[neighbour[0]][0]:
                                skip = 1

                        if skip == 0:
                            heapq.heappush(open_heap, (total_cost, neighbour[0]))
                            open_dict[neighbour[0]] = (total_cost, neighbour[1], (chosen_d_node, chosen_c_node), (velocity, steering_angle))

        print("count: ", count)
        print("Found nothing???")
        return None

if __name__ == "__main__":
    hybrid_astar = Hybrid_AStar()
    path, states = hybrid_astar.run()

    x = [i[1][0] for i in path]
    y = [i[1][1] for i in path]

    plt.plot(x, y)
    plt.grid()
    plt.show()








