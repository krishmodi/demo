from enum import Enum
import time

class Action(Enum):
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
    UP = 'UP'
    DOWN = 'DOWN'

class Node:
    def __init__(self, state, parent=None, action=None, depth=0, cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = depth
        self.cost = cost

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return (self.cost + self.depth) < (other.cost + other.depth)

    def __hash__(self):
        return hash(str(self.state))

class AStarSearch:
    def __init__(self, initial_state, goal_state):
        self.initial_node = Node(initial_state)
        self.goal_state = goal_state
        self.start_time = None
        self.total_actions = 0

    def get_blank_position(self, state):
        for i in range(3):
            for j in range(3):
                if state[i][j] == '_':
                    return i, j

    def get_children(self, node):
        children = []
        row, col = self.get_blank_position(node.state)

        # Generate child nodes by applying possible actions
        for action in Action:
            new_state = [row[:] for row in node.state]
            if action == Action.LEFT and col > 0:
                new_state[row][col], new_state[row][col - 1] = new_state[row][col - 1], new_state[row][col]
                children.append(Node(new_state, node, action, node.depth + 1, 0))
            elif action == Action.RIGHT and col < 2:
                new_state[row][col], new_state[row][col + 1] = new_state[row][col + 1], new_state[row][col]
                children.append(Node(new_state, node, action, node.depth + 1, 0))
            elif action == Action.UP and row > 0:
                new_state[row][col], new_state[row - 1][col] = new_state[row - 1][col], new_state[row][col]
                children.append(Node(new_state, node, action, node.depth + 1, 0))
            elif action == Action.DOWN and row < 2:
                new_state[row][col], new_state[row + 1][col] = new_state[row + 1][col], new_state[row][col]
                children.append(Node(new_state, node, action, node.depth + 1, 0))
        return children

    def heuristic(self, state):
        # Manhattan distance heuristic
        distance = 0
        for i in range(3):
            for j in range(3):
                if state[i][j] != self.goal_state[i][j] and state[i][j] != '_':
                    value = state[i][j]
                    for row in range(3):
                        for col in range(3):
                            if self.goal_state[row][col] == value:
                                distance += abs(i - row) + abs(j - col)
        return distance

    def perform_algorithm(self):
        self.start_time = time.time()
        open_list = [self.initial_node]
        closed_list = set()

        while open_list:
            current_node = min(open_list)
            open_list.remove(current_node)
            closed_list.add(current_node)

            if current_node.state == self.goal_state:
                return self.get_path(current_node)

            children = self.get_children(current_node)
            for child in children:
                if child not in closed_list:
                    child.cost = self.heuristic(child.state)
                    open_list.append(child)
                    self.total_actions += 1

        return None

    def get_path(self, node):
        path = []
        while node:
            path.append((node.state, node.action))
            node = node.parent
        path.reverse()
        return path

def print_state(state):
    for row in state:
        print(row)

# Test cases
initial_state = [[1, 2, 3],
                 [4, '_', 5],
                 [6, 7, 8]]

goal_state = [[1, 2, 3],
              [4, 5, 6],
              [7, 8, '_']]

print("Initial state:")
print_state(initial_state)
print("Goal state:")
print_state(goal_state)

solver = AStarSearch(initial_state, goal_state)
path = solver.perform_algorithm()

if path:
    print("Solution:")
    for state, action in path:
        if action:
            print("Action:", action.value)
        print_state(state)
    print("Total actions performed:", solver.total_actions)
else:
    print("No solution found.")

end_time = time.time()
print("Execution time:", end_time - solver.start_time, "seconds")
