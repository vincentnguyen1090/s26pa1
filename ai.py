from __future__ import print_function
from heapq import *  # Hint: Use heappop and heappush
from collections import deque

ACTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]


class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = deque([self.grid.start])
        elif self.type == "ucs":
            # we will keep the smallest node at the top
            self.frontier = []
            # push (cost, node)
            heappush(self.frontier, (0, self.grid.start))
            self.cost = {self.grid.start: 0}
        elif self.type == "astar":
            pass

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            if self.type == "bfs":
                total_cost += 1
            else:
                total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = (
                True  # This turns the color of the node to red
            )
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    # TODO: Buggy DFS, fix it first
    # Issue: we append every non-puddle child to the frontier
    # We should not append if the child already in:
    # 1. frontier
    # 2. visited/explored
    def dfs_step(self):
        # If nothing left to explore, no path found
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        # Popping last-added node from the stack
        current = self.frontier.pop()

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        # create all neighboring positions
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True    # mark current node as explored
        self.grid.nodes[current].color_frontier = False  # mark colored as a frontier node

        # Go through every child node
        for n in children:
            # Check if chid within boundaries
            if n[0] in range(self.grid.row_range) and n[1] in range(
                self.grid.col_range
            ):
                # Skip puddles
                if self.grid.nodes[n].puddle:
                    continue
                
                # Skip nodes already explored / in frontier
                if self.grid.nodes[n].color_checked or self.grid.nodes[n].color_frontier:
                    continue
                
                self.previous[n] = current                # Keep track of parent node -> reconstuct path later
                self.frontier.append(n)                   # Add child to frontier
                self.grid.nodes[n].color_frontier = True  # Mark child as added to frontier

    # TODO: Implement BFS here (Don't forget to implement initialization in set_search function)
    def bfs_step(self):
        if not self.frontier:            
            self.failed = True
            self.finished = True
            print("no path")
            return
        # popping the first node in queue
        current = self.frontier.popleft()
        
         # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return


        # create all neighboring positions
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True    # mark current node as explored
        self.grid.nodes[current].color_frontier = False  # mark colored as a frontier node
        
        # Go through every child node
        for n in children:
            # Check if chid within boundaries
            if n[0] in range(self.grid.row_range) and n[1] in range(
                self.grid.col_range
            ):
                # Skip puddles
                if self.grid.nodes[n].puddle:
                    continue
                
                # Skip nodes already explored / in frontier
                if self.grid.nodes[n].color_checked or self.grid.nodes[n].color_frontier:
                    continue
                
                self.previous[n] = current                # Keep track of parent node -> reconstuct path later
                self.frontier.append(n)                   # Add child to frontier
                self.grid.nodes[n].color_frontier = True  # Mark child as added to frontier

    # TODO: Implement UCS here (Don't forget to implement initialization in set_search function)
    # Hint: You can use heappop and heappush from the heapq library (imported for you above)
    def ucs_step(self):
        # If nothing left to explore, no path found
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        # cost to get to this node | current_node
        cost, current = heappop(self.frontier)
        
        # Skip outdated heap entries -> we've already found cheaper path to this node
        if cost > self.cost[current]:
            return
        
        if current == self.grid.goal:
            self.finished = True
            self.final_cost = cost
            return
            
        
        # create all neighboring positions
        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True    # mark current node as explored
        self.grid.nodes[current].color_frontier = False  # mark colored as a frontier node
        
        for n in children:
            # Check if chid within boundaries
            if n[0] in range(self.grid.row_range) and n[1] in range(
                self.grid.col_range
            ):
                if self.grid.nodes[n].puddle:
                        continue
                
                new_cost = cost + 1
                
                # 1. Never seen this node before (take this path) OR 
                # 2. We have seen this node, check if there's a cheaper path
                if n not in self.cost or new_cost < self.cost[n]:
                    self.cost[n] = new_cost                     # Update new cost for this node
                    heappush(self.frontier, (new_cost, n))      # Push new_cost, node into min_heap
                    self.previous[n] = current                  # Keep track of parent node -> reconstuct path later
                    self.grid.nodes[n].color_frontier = True    # Mark child as added to frontier

    # TODO: Implement Astar here (Don't forget to implement initialization in set_search function)
    # Hint: You can use heappop and heappush from the heapq library (imported for you above)
    def astar_step(self):
        self.failed = True
        self.finished = True
