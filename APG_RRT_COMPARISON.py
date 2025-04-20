import matplotlib.pyplot as plt
import numpy as np
import random
import time

# Node class
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

def is_collision_free(n1, n2, obstacles, buffer=1):
    points = np.linspace([n1.x, n1.y], [n2.x, n2.y], 100)
    for x, y in points:
        for dx in range(-buffer, buffer + 1):
            for dy in range(-buffer, buffer + 1):
                xi, yi = int(x + dx), int(y + dy)
                if 0 <= xi < obstacles.shape[1] and 0 <= yi < obstacles.shape[0]:
                    if obstacles[yi, xi]:
                        return False
    return True

def backtrack_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def generate_guide_paths(scenario='u_turn'):
    if scenario == 'u_turn':
        return [[Node(10 + i, 50) for i in range(15)] +
                [Node(25, 50 + i*2) for i in range(5)] +
                [Node(40 + i, 60) for i in range(10)]]
    elif scenario == 'long_corridor':
        return [[Node(x, 50) for x in range(10, 90, 5)]]
    elif scenario == 'right_angle_bends':
        return [[Node(10, 30), Node(20, 30), Node(20, 50), Node(40, 50)]]
    else:
        return [[Node(x, 50) for x in range(0, 100, 10)]]


def get_random_point(map_size, obstacles):
    while True:
        x, y = random.randint(0, map_size[0] - 1), random.randint(0, map_size[1] - 1)
        if not obstacles[y, x]:
            return Node(x, y)

def nearest_node(nodes, point):
    return min(nodes, key=lambda node: distance(node, point))

def compute_path_length(path):
    if not path or len(path) < 2:
        return 0
    return sum(np.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(path[:-1], path[1:]))

def rrt(start, goal, map_size, obstacles, max_iterations=10000, step_size=5):
    nodes = [start]
    for _ in range(max_iterations):
        rand = get_random_point(map_size, obstacles)
        nearest = nearest_node(nodes, rand)
        new_node = steer(nearest, rand, step_size)
        if 0 <= new_node.x < map_size[0] and 0 <= new_node.y < map_size[1]:
            if is_collision_free(nearest, new_node, obstacles):
                new_node.parent = nearest
                nodes.append(new_node)
                if distance(new_node, goal) < step_size:
                    goal.parent = new_node
                    return backtrack_path(goal), len(nodes)
    return None, len(nodes)

def apg_rrt(start, goal, map_size, obstacles, max_iter=2000, max_dist=10, buffer=1, alpha=1.0, beta=1.0):
    guide_paths = generate_guide_paths()
    weights = [1 / len(guide_paths)] * len(guide_paths)
    root = Node(*start)
    goal_node = Node(*goal)
    nodes = [root]

    for i in range(max_iter):
        if random.random() < 0.1:
            idx = random.choices(range(len(guide_paths)), weights=weights)[0]
            path = guide_paths[idx][:15]
            success = True
            for node in path:
                nearest = nearest_node(nodes, node)
                new_node = steer(nearest, node, max_dist)
                if is_collision_free(nearest, new_node, obstacles, buffer):
                    new_node.parent = nearest
                    nodes.append(new_node)
                else:
                    weights[idx] *= np.exp(-alpha)
                    success = False
                    break
            if success:
                weights[idx] *= np.exp(beta)
        else:
            rnd = get_random_point(map_size, obstacles)
            nearest = nearest_node(nodes, rnd)
            new_node = steer(nearest, rnd, max_dist)
            if is_collision_free(nearest, new_node, obstacles, buffer):
                new_node.parent = nearest
                nodes.append(new_node)

        # Normalize weights
        total = sum(weights)
        weights = [w / total for w in weights]

        for node in nodes:
            if distance(node, goal_node) <= max_dist and is_collision_free(node, goal_node, obstacles, buffer):
                goal_node.parent = node
                print("✅ APG-RRT Goal reached!")
                return backtrack_path(goal_node), len(nodes)

    print("❌ APG-RRT No path found.")
    return None, len(nodes)

def create_obstacle_map(size, scenario='long_corridor'):
    grid = np.zeros(size, dtype=int)
    if scenario == 'long_corridor':
        # grid[30:70, 40:60] = 1    # Vertical corridor walls
        grid[0:3, :] = 1       # Top wall
        grid[97:100, :] = 1       # Bottom wall
        grid[10:100, 30:40] = 1
        grid[0:90, 70:80] = 1

    elif scenario == 'right_angle_bends':
        grid[20:60, 40:45] = 1   # Vertical wall
        grid[55:60, 40:80] = 1   # Horizontal wall
        grid[30:35, 10:50] = 1   # Another horizontal section
        grid[60:80, 70:75] = 1   # Another vertical block

    elif scenario == 'u_turn':

        # grid[10:20, 0:90] = 1
        grid[30:40, 10:100] = 1
        grid[50:60, 0:90] = 1
        grid[70:80, 10:100] = 1
        

    else:  # Default mixed environment
        grid[30:70, 40:60] = 1    # Vertical block
        grid[10:20, 10:50] = 1    # Horizontal corridor
        grid[70:90, 20:30] = 1    # Small vertical block
        grid[50:55, 60:100] = 1   # Horizontal barrier
        grid[20:30, 70:80] = 1    # Isolated block
        grid[60:70, 5:15] = 1     # Another isolated block
    return grid

def visualize(map_size, obstacles, rrt_path, apg_path):
    plt.figure(figsize=(6, 6))
    plt.imshow(1 - obstacles, cmap='gray', origin='lower')
    if rrt_path:
        rrt_path = np.array(rrt_path)
        plt.plot(rrt_path[:, 0], rrt_path[:, 1], 'r-', label='RRT Path')
    if apg_path:
        apg_path = np.array(apg_path)
        plt.plot(apg_path[:, 0], apg_path[:, 1], 'g-', label='APG-RRT Path')
    plt.legend()
    plt.title("Path Comparison")
    plt.show()

# Main execution
map_size = (100, 100)
obstacles = create_obstacle_map(map_size)

start = Node(5, 5)
goal = Node(95, 95)

start_time = time.time()
rrt_path, rrt_nodes = rrt(start, goal, map_size, obstacles)
rrt_time = time.time() - start_time
rrt_length = compute_path_length(rrt_path)

start_time = time.time()
apg_path, apg_nodes = apg_rrt((start.x, start.y), (goal.x, goal.y), map_size, obstacles)
apg_time = time.time() - start_time
apg_length = compute_path_length(apg_path)

print(f"RRT Time: {rrt_time:.4f}s")
print(f"RRT Path Length: {rrt_length:.2f}")
print(f"RRT Nodes Used: {rrt_nodes}")

print(f"APG-RRT Time: {apg_time:.4f}s")
print(f"APG-RRT Path Length: {apg_length:.2f}")
print(f"APG-RRT Nodes Used: {apg_nodes}")

visualize(map_size, obstacles, rrt_path, apg_path)