import random
import time

from pygame import init

import config
from collections import deque
import heapq


class Algorithm:
    def __init__(self, heuristic=None):
        self.heuristic = heuristic
        self.nodes_evaluated = 0
        self.nodes_generated = 0

    # returns all possible actions that we can perform to get us into another state
    def get_legal_actions(self, state):
        self.nodes_evaluated += 1
        max_index = len(state)
        zero_tile_ind = state.index(0)
        legal_actions = []
        if 0 <= (up_ind := (zero_tile_ind - config.N)) < max_index:
            legal_actions.append(up_ind)
        if 0 <= (right_ind := (zero_tile_ind + 1)) < max_index and right_ind % config.N:
            legal_actions.append(right_ind)
        if 0 <= (down_ind := (zero_tile_ind + config.N)) < max_index:
            legal_actions.append(down_ind)
        if 0 <= (left_ind := (zero_tile_ind - 1)) < max_index and (left_ind + 1) % config.N:
            legal_actions.append(left_ind)
        return legal_actions

    # applies a single action to our state
    def apply_action(self, state, action):
        self.nodes_generated += 1
        copy_state = list(state)
        zero_tile_ind = state.index(0)
        copy_state[action], copy_state[zero_tile_ind] = copy_state[zero_tile_ind], copy_state[action]
        return tuple(copy_state)

    def get_steps(self, initial_state, goal_state):
        pass

    def get_solution_steps(self, initial_state, goal_state):
        begin_time = time.time()
        solution_actions = self.get_steps(initial_state, goal_state)
        print(f'Execution time in seconds: {(time.time() - begin_time):.2f} | '
              f'Nodes generated: {self.nodes_generated} | '
              f'Nodes evaluated: {self.nodes_evaluated}')
        return solution_actions


class ExampleAlgorithm(Algorithm):
    def get_steps(self, initial_state, goal_state):
        state = initial_state
        solution_actions = []
        while state != goal_state:
            legal_actions = self.get_legal_actions(state)
            action = legal_actions[random.randint(0, len(legal_actions) - 1)]
            solution_actions.append(action)
            state = self.apply_action(state, action)
        return solution_actions


class BFSAlgorithm(Algorithm):
    def get_steps(self, initial_state, goal_state):

        nodes = deque([(initial_state, [])])
        visited = set()

        while nodes:

            current_state, actions = nodes.popleft()
            solution_actions = self.get_legal_actions(current_state)
            if current_state == goal_state:
                return actions
            for action in solution_actions:
                new_state = self.apply_action(current_state, action)
                if new_state not in visited:
                    nodes.append((new_state, actions + [action]))
                    visited.add(new_state)
        return []


class BestFirstAlgorithm(Algorithm):
    def get_steps(self, initial_state, goal_state):
        nodes = []

        heuristic_value = self.heuristic.get_evaluation(initial_state, goal_state)
        node = (initial_state, [])
        heapq.heappush(nodes, (heuristic_value, node))  # actions are key and the priority is
        visited = set()

        while nodes:
            current_state, current_action = heapq.heappop(nodes)[1]  # access the node
            visited.add(current_state)

            if current_state == goal_state:
                return current_action

            legal_actions = self.get_legal_actions(current_state)

            for action in legal_actions:
                next_state = self.apply_action(current_state, action)

                if next_state not in visited:
                    next_node = (next_state, current_action + [action])
                    next_heuristic_value = self.heuristic.get_evaluation(next_state, goal_state)
                    heapq.heappush(nodes, (next_heuristic_value, next_node))
        return []


class AStarAlgorithm(Algorithm):
    def get_steps(self, initial_state, goal_state):
        visited = set()
        nodes = []
        heuristic_value = self.heuristic.get_evaluation(initial_state, goal_state)
        initial_node = (initial_state, [], 0)  # state, actions, summed cost
        heapq.heappush(nodes, (heuristic_value, initial_node))

        while nodes:
            current_state, current_actions, current_cost = heapq.heappop(nodes)[1]
            if current_state == goal_state:
                return current_actions

            legal_actions = self.get_legal_actions(current_state)

            for action in legal_actions:
                new_state = self.apply_action(current_state, action)
                if new_state not in visited:
                    new_heuristic_value = self.heuristic.get_evaluation(new_state, goal_state)
                    new_g = current_cost + 1
                    new_node = (new_state, current_actions + [action], new_g)
                    new_cost = new_heuristic_value + new_g
                    visited.add(new_state)
                    heapq.heappush(nodes, (new_cost, new_node))
        return []
