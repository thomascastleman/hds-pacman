# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

# check if two tuples are equal
def checkTupleEquality(t1, t2):
    return set(t1) == set(t2)

def depthFirstSearch(problem):
        
    start = [problem.getStartState()]       # get initial state
    stack = util.Stack()                    # stack for DFS
    visited = set()                         # set of all (x, y) coords already seen
    parents = {}                            # dictionary of children to parent

    from copy import deepcopy   
    current = deepcopy(start)       # current node starts at start state
    stack.push(current)             # add current to stack

    while not stack.isEmpty():

        current = stack.pop()

        # if at goal state, end
        if problem.isGoalState(current[0]):
            break
        else:

            # get all successor states
            children = problem.getSuccessors(current[0])
            for child in children:

                # if not already explored
                if child[0] not in visited:
                    # add to necessary structures
                    stack.push(child)
                    visited.add(child[0])
                    parents[child] = current

    # if no path found, throw error
    if not problem.isGoalState(current[0]):
        util.raiseNotDefined()


    # backtrack and reconstruct path
    path = []
    while not checkTupleEquality(current, start):
        path.insert(0, current[1])
        current = parents[current]

    return path

def breadthFirstSearch(problem):

    start = [problem.getStartState()]       # get start state from problem
    q = util.Queue()                        # queue for BFS
    visited = set()                         # set of all (x, y) coords already seen
    parents = {}                            # dictionary of children to parent

    from copy import deepcopy
    current = deepcopy(start)       # copy start
    q.push(current)                 # enqueue

    while not q.isEmpty():

        # pop from queue
        current = q.pop()

        # if goal found, end
        if problem.isGoalState(current[0]):
            break
        else:
            # get all successors
            children = problem.getSuccessors(current[0])

            for child in children:
                # if not already seen
                if child[0] not in visited:
                    # add to necessary structures
                    q.push(child)
                    visited.add(child[0])
                    parents[child] = current

    # if no path found, throw error
    if not problem.isGoalState(current[0]):
        util.raiseNotDefined()


    # trace back, reconstruct path
    path = []
    while not checkTupleEquality(current, start):
        path.insert(0,current[1])
        current = parents[current]

    return path

def uniformCostSearch(problem):

    # start = problem.getStartState()
    # pq = PriorityQueue()
    # visited = set()
    # parents = {}

    # from copy import deepcopy
    # current = deepcopy(start)
    # pq.push(current)

    # while not pq.isEmpty():
    #     current = pq.pop()
    #     if problem.isGoalState(current):
    #         break
    #     else:
    #         children = problem.getSuccessors(current)
    #         for child in children:
    #             if child not in visited:
    #                 pq.push(child)
    #                 visited.add(child)
    #                 parents[child] = current

    # if not current.isGoalState():
    #     util.raiseNotDefined()


    # path = []

    # while not checkTupleEquality(current, start):
    #     path.insert(0,current[1])
    #     current = parents[current]

    # return path

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):


    # use problem.costFn( tuple ) for g cost

    start = [problem.getStartState()]
    pq = PriorityQueue()
    visited = set()
    parents = {}

    from copy import deepcopy
    current = deepcopy(start)

    # not sure about this
    pq.push(current, heuristic(current, problem))

    while not pq.isEmpty():
        current = pq.pop()
        if problem.isGoalState(current[0]):
            break
        else:
            children = problem.getSuccessors(current[0])
            for child in children:
                if child not in visited:
                    pq.push(child, self.costFn(child[0]) + heuristic(child, problem))
                    visited.add(child)
                    parents[child] = current

    if not current.isGoalState():
        util.raiseNotDefined()


    path = []

    while not checkTupleEquality(current, start):
        path.insert(0,current[1])
        current = parents[current]

    return path











# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
