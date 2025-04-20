
import numpy as np
import matplotlib.pyplot as plt
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(n1, n2):
    return np.hypot(n1.x - n2.x, n1.y - n2.y)

def steer(from_node, to_node, max_dist):
    d = distance(from_node, to_node)
    if d > max_dist:
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        return Node(from_node.x + max_dist * np.cos(theta),
                    from_node.y + max_dist * np.sin(theta))
    return to_node

def is_collision_free(n1, n2, obstacles, buffer=3):
    points = np.linspace([n1.x, n1.y], [n2.x, n2.y], 20)
    for x, y in points:
        for ox, oy, r in obstacles:
            if np.hypot(x - ox, y - oy) <= r + buffer:
                return False
    return True

def backtrack_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def generate_guide_paths():
    straight = [Node(x, 50) for x in range(0, 100, 10)]
    bend = [Node(20 + i, 50) for i in range(5)] + [Node(25, 50 + i*5) for i in range(5)]
    uturn = [Node(20 + i, 40) for i in range(5)] + [Node(25, 40 + i*2) for i in range(10)] + [Node(30 + i, 60) for i in range(5)]
    return [straight, bend, uturn]

def apg_rrt(start, goal, space, obstacle_list, max_iter=200, max_dist=10, buffer=3, alpha=1.0, beta=1.0, max_nodes=1000):
    guide_paths = generate_guide_paths()
    weights = [1/len(guide_paths)] * len(guide_paths)
    root = Node(*start)
    goal_node = Node(*goal)
    nodes = [root]

    for i in range(max_iter):

        if len(nodes) > max_nodes:
            print("Node limit reached, stopping early.")
            break

        if random.random() < 0.6:
            selected_idx = random.choices(range(len(guide_paths)), weights=weights)[0]
            path = guide_paths[selected_idx][:5]
            success = True
            for node in path:
                nearest = min(nodes, key=lambda n: distance(n, node))
                new_node = steer(nearest, node, max_dist)
                if is_collision_free(nearest, new_node, obstacle_list, buffer):
                    new_node.parent = nearest
                    nodes.append(new_node)
                    if len(nodes) > max_nodes:
                        break
                else:
                    weights[selected_idx] *= np.exp(-alpha)
                    success = False
                    break
            if success:
                weights[selected_idx] *= np.exp(beta)
        else:
            rnd_node = Node(random.uniform(0, space[0]), random.uniform(0, space[1]))
            nearest = min(nodes, key=lambda n: distance(n, rnd_node))
            new_node = steer(nearest, rnd_node, max_dist)
            if is_collision_free(nearest, new_node, obstacle_list, buffer):
                new_node.parent = nearest
                nodes.append(new_node)

        # Normalize weights
        total = sum(weights)
        weights = [w / total for w in weights]

        for node in nodes:
            if distance(node, goal_node) <= max_dist and is_collision_free(node, goal_node, obstacle_list, buffer):
                goal_node.parent = node
                print("Goal reached!")
                return backtrack_path(goal_node)

    print("No path found.")
    return None

def plot_result(path, start, goal, obstacles, space):
    plt.figure(figsize=(6, 6))
    plt.title("Final Path")
    plt.xlim(0, space[0])
    plt.ylim(0, space[1])
    plt.gca().set_aspect('equal')

    for ox, oy, r in obstacles:
        circle = plt.Circle((ox, oy), r, color='black')
        plt.gca().add_patch(circle)

    if path:
        x, y = zip(*path)
        plt.plot(x, y, 'r-', linewidth=2, label="Path")
    else:
        print("No path to plot.")

    plt.plot(start[0], start[1], 'go', label='Start')
    plt.plot(goal[0], goal[1], 'ro', label='Goal')
    plt.legend()
    plt.show()

# Test run
space = (100, 100)
start = (5, 5)
goal = (90, 90)
obstacle_list = [(40, 40, 10), (60, 60, 10), (20, 80, 10), (70, 20, 10)]

path = apg_rrt(start, goal, space, obstacle_list)
plot_result(path, start, goal, obstacle_list, space)
