class Heuristic:

    def get_evaluation(self, state, goal_state):
        pass


class ExampleHeuristic(Heuristic):
    def get_evaluation(self, state, goal_state):
        return 0


class HammingHeuristic(Heuristic):

    def get_evaluation(self, state, goal_state):

        hamming_distance = 0
        for i in range(len(state)):
            if state[i] != goal_state[i] and goal_state[i] != 0:
                hamming_distance += 1
        # hamming_distance = sum(s != g for s, g in zip(state, goal_state))
        return hamming_distance


class ManhattanHeuristic(Heuristic):

    def get_evaluation(self, state, goal_state):
        manhattan_distance = 0
        size = int(len(state) ** 0.5)

        for tile in range(size * size):
            current_tile = goal_state[tile]
            goal_tile = state.index(current_tile)

            if goal_state[tile] != 0:
                i, j = divmod(tile, size)
                p, q = divmod(goal_tile, size)

                manhattan_distance += (abs(i - p) + abs(j - q))

        return manhattan_distance
