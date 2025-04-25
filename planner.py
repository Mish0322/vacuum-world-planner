import sys

### ----- State Class -----
class State:
    def __init__(self, position, dirty):
        self.position = position              # (row, col)
        self.dirty = frozenset(dirty)         # remaining dirty cells

    def is_goal(self):
        return len(self.dirty) == 0

    def __hash__(self):
        return hash((self.position, self.dirty))

    def __eq__(self, other):
        return self.position == other.position and self.dirty == other.dirty


### ----- Read Grid From File -----
def load_world(file_path):
    with open(file_path, 'r') as f:
        cols = int(f.readline())
        rows = int(f.readline())
        grid = [list(f.readline().strip()) for _ in range(rows)]
    return rows, cols, grid


### ----- Find Robot Start + Dirty Cells -----
def find_robot_and_dirt(grid):
    dirty = set()
    robot_pos = None
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] == '@':
                robot_pos = (r, c)
            elif grid[r][c] == '*':
                dirty.add((r, c))
    return robot_pos, dirty


### ----- Get Successor States -----
def get_successors(state, grid):
    rows, cols = len(grid), len(grid[0])
    r, c = state.position
    dirty = set(state.dirty)
    successors = []

    directions = {
        'N': (-1, 0),
        'S': (1, 0),
        'E': (0, 1),
        'W': (0, -1)
    }

    # Move in 4 directions if not blocked
    for action, (dr, dc) in directions.items():
        new_r, new_c = r + dr, c + dc
        if 0 <= new_r < rows and 0 <= new_c < cols and grid[new_r][new_c] != '#':
            successors.append((State((new_r, new_c), dirty), action, 1))

    # Vacuum if on dirty cell
    if state.position in dirty:
        new_dirty = set(dirty)
        new_dirty.remove(state.position)
        successors.append((State(state.position, new_dirty), 'V', 1))

    return successors

from collections import deque

def dfs(start_state, grid):
    stack = [(start_state, [])]
    visited = set()
    nodes_generated = 0
    nodes_expanded = 0

    while stack:
        current_state, path = stack.pop()

        if current_state in visited:
            continue

        visited.add(current_state)
        nodes_expanded += 1

        if current_state.is_goal():
            return path, nodes_generated, nodes_expanded

        for successor, action, cost in get_successors(current_state, grid):
            if successor not in visited:
                stack.append((successor, path + [action]))
                nodes_generated += 1

    return None, nodes_generated, nodes_expanded  # no solution found

import heapq

def ucs(start_state, grid):
    frontier = []
    counter = 0 # to resolve the issue when it compares 2 actions with the same cost
    heapq.heappush(frontier, (0, counter, start_state, []))
    cost_so_far = {start_state: 0}
    nodes_generated = 0
    nodes_expanded = 0

    while frontier:
        current_cost, _, current_state, path = heapq.heappop(frontier)

        if current_state.is_goal():
            return path, nodes_generated, nodes_expanded

        nodes_expanded += 1

        for successor, action, step_cost in get_successors(current_state, grid):
            new_cost = current_cost + step_cost

            if successor not in cost_so_far or new_cost < cost_so_far[successor]:
                cost_so_far[successor] = new_cost
                counter += 1
                heapq.heappush(frontier, (new_cost, counter, successor, path + [action]))
                nodes_generated += 1

    return None, nodes_generated, nodes_expanded

### ----- Main Program Entry Point -----
def main():
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py [uniform-cost|depth-first] [world-file]")
        return

    algorithm = sys.argv[1]
    file_path = sys.argv[2]

    rows, cols, grid = load_world(file_path)
    robot_pos, dirty = find_robot_and_dirt(grid)

    initial_state = State(robot_pos, dirty)

    if algorithm == "depth-first":
        plan, generated, expanded = dfs(initial_state, grid)
    elif algorithm == "uniform-cost":
        plan, generated, expanded = ucs(initial_state, grid)
    else:
        print("Unknown algorithm:", algorithm)
        return
    
    if plan is None:
        print("No solution found.")
    else:
        for action in plan:
            print(action)
        print(f"{generated} nodes generated")
        print(f"{expanded} nodes expanded")

if __name__ == "__main__":
    main()
