# Solving a given maze
# Authors:
# Roaa Fathi
# Selsabeel Asim

# Start State (0, 0)
# Goal State (12, 10)

# In the Given Maze:
# 0 means a Node
# 1 means an Edge
# 4 means a Horizontal wall
# 5 means a Vertical wall

from collections import deque
from enum import Enum
from typing import List, NamedTuple, Callable, Optional
from math import sqrt
from heapq import heappop, heappush


class Node(NamedTuple):
    state: 'MazeLocation'
    parent: Optional['Node']
    cost: float = 0.0
    heuristic: float = 0.0

    def __lt__(self, other: 'Node'):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)


def dfs(initial, goal_test, successors):
    frontier = [Node(initial, None)]
    explored = {initial}

    while frontier:
        current_node = frontier.pop()
        current_state = current_node.state

        if goal_test(current_state):
            return current_node

        for child in successors(current_state):
            if child in explored:
                continue
            explored.add(child)
            frontier.append(Node(child, current_node))

    return None


def bfs(initial, goal_test, successors):
    frontier = deque([Node(initial, None)])
    explored = {initial}

    while frontier:
        current_node = frontier.popleft()
        current_state = current_node.state

        if goal_test(current_state):
            return current_node

        for child in successors(current_state):
            if child in explored:
                continue
            explored.add(child)
            frontier.append(Node(child, current_node))

    return None


def astar(initial, goal_test, successors, heuristic):
    frontier = []
    heappush(frontier, Node(initial, None, 0.0, heuristic(initial)))
    explored = {initial: 0.0}

    while frontier:
        current_node = heappop(frontier)
        current_state = current_node.state

        if goal_test(current_state):
            return current_node

        for child in successors(current_state):
            new_cost = current_node.cost + 1  # Assume cost between nodes is 1
            if child not in explored or explored[child] > new_cost:
                explored[child] = new_cost
                heappush(frontier, Node(child, current_node, new_cost, heuristic(child)))

    return None


def node_to_path(node):
    path = [node.state]
    while node.parent is not None:
        node = node.parent
        path.append(node.state)
    path.reverse()
    return path


class Cell(str, Enum):
    Empty = " "
    Blocked = "x"
    Start_State = "S"
    Goal_State = "G"
    Path = "="


class MazeLocation(NamedTuple):
    row: int
    column: int


class Maze:
    def __init__(self, custom_grid, start, goal):
        self._rows = len(custom_grid)
        self._columns = len(custom_grid[0])
        self.start = start
        self.goal = goal
        self._grid = self._fill_customized_grid(custom_grid)
        self._grid[start.row][start.column] = Cell.Start_State
        self._grid[goal.row][goal.column] = Cell.Goal_State

    def _fill_customized_grid(self, customized_grid):
        custom_grid = []
        for row in customized_grid:
            custom_row = []
            for cell in row:
                if cell == " ":
                    custom_row.append(Cell.Empty)
                else:
                    custom_row.append(Cell.Blocked)
            custom_grid.append(custom_row)
        return custom_grid

    def __str__(self):
        output = ""
        for row in self._grid:
            output += "".join([c.value for c in row]) + "\n"
        return output

    def goal_test(self, maze_location):
        return maze_location == self.goal

    def get_available_moves(self, ml):
        locations = []
        # below cell
        if ml.row + 1 < self._rows and self._grid[ml.row + 1][ml.column] != Cell.Blocked:
            locations.append(MazeLocation(ml.row + 1, ml.column))
        # above cell
        if ml.row - 1 >= 0 and self._grid[ml.row - 1][ml.column] != Cell.Blocked:
            locations.append(MazeLocation(ml.row - 1, ml.column))
        # right cell
        if ml.column + 1 < self._columns and self._grid[ml.row][ml.column + 1] != Cell.Blocked:
            locations.append(MazeLocation(ml.row, ml.column + 1))
        # left cell
        if ml.column - 1 >= 0 and self._grid[ml.row][ml.column - 1] != Cell.Blocked:
            locations.append(MazeLocation(ml.row, ml.column - 1))
        return locations

    def highlight_path(self, path):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.Path

        self._grid[self.start.row][self.start.column] = Cell.Start_State
        self._grid[self.goal.row][self.goal.column] = Cell.Goal_State

    def clear_highlighted_path(self, path):
        for maze_location in path:
            self._grid[maze_location.row][maze_location.column] = Cell.Empty

        self._grid[self.start.row][self.start.column] = Cell.Start_State
        self._grid[self.goal.row][self.goal.column] = Cell.Goal_State

    def path_cost(self):
        return sum(row.count(Cell.Path) for row in self._grid)


def euclidean_distance(goal):
    def distance(ml):
        x_axis_distance = ml.column - goal.column
        y_axis_distance = ml.row - goal.row
        return sqrt((y_axis_distance * y_axis_distance) + (x_axis_distance * x_axis_distance))
    return distance


def manhattan_distance(goal):
    def distance(ml):
        x_axis_distance = abs(ml.column - goal.column)
        y_axis_distance = abs(ml.row - goal.row)
        return x_axis_distance + y_axis_distance
    return distance


def convert_to_custom_maze(maze):
    custom_maze = []
    for row in maze:
        custom_row = []
        for cell in row:
            if cell == 0 or cell == 1 or cell == 9:
                custom_row.append(" ")
            else:
                custom_row.append("x")
        custom_maze.append(custom_row)
    return custom_maze


# Given maze
maze = [
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 5, 0],
    [1, 9, 4, 9, 1, 9, 1, 9, 4, 9, 1],
    [0, 1, 0, 5, 0, 5, 0, 5, 0, 5, 0],
    [1, 9, 4, 9, 1, 9, 4, 9, 1, 9, 1],
    [0, 1, 0, 5, 0, 1, 0, 1, 0, 5, 0],
    [1, 9, 1, 9, 4, 9, 4, 9, 1, 9, 1],
    [0, 5, 0, 5, 0, 5, 0, 1, 0, 1, 0],
    [1, 9, 4, 9, 1, 9, 1, 9, 1, 9, 1],
    [0, 1, 0, 5, 0, 1, 0, 5, 0, 5, 0],
    [4, 9, 1, 9, 1, 9, 4, 9, 4, 9, 1],
    [0, 5, 0, 5, 0, 5, 0, 5, 0, 1, 0],
    [1, 9, 1, 9, 1, 9, 1, 9, 4, 9, 1],
    [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0],
]

custom_maze = convert_to_custom_maze(maze)

if __name__ == "__main__":
    start = MazeLocation(0, 0)
    goal = MazeLocation(12, 10)

    given_maze = Maze(custom_maze, start, goal)
    print("******* Customized Maze ******")
    print(given_maze)

    print("=================== DFS ==========================")
    solution1 = dfs(given_maze.start, given_maze.goal_test, given_maze.get_available_moves)
    if solution1 is None:
        print("No solution found using DFS!")
    else:
        path1 = node_to_path(solution1)
        given_maze.highlight_path(path1)
        print("DFS Path:")
        print(given_maze)
        print(f"DFS Path Cost: {given_maze.path_cost()}")
        given_maze.clear_highlighted_path(path1)

    print("=================== BFS ===========================")
    solution2 = bfs(given_maze.start, given_maze.goal_test, given_maze.get_available_moves)
    if solution2 is None:
        print("No solution found using BFS!")
    else:
        path2 = node_to_path(solution2)
        given_maze.highlight_path(path2)
        print("BFS Path:")
        print(given_maze)
        print(f"BFS Path Cost: {given_maze.path_cost()}")
        given_maze.clear_highlighted_path(path2)

    print("==================  A*  ========================= ")
    distance = manhattan_distance(given_maze.goal)
    solution3 = astar(given_maze.start, given_maze.goal_test, given_maze.get_available_moves, distance)
    if solution3 is None:
        print("No solution found using A*!")
    else:
        path3 = node_to_path(solution3)
        given_maze.highlight_path(path3)
        print("A* Path:")
        print(given_maze)
        print(f"A* Path Cost: {given_maze.path_cost()}")
