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

# error for no path found
def throwNoPathError():
    print "----- ERROR: NO PATH FOUND -----"
    sys.exit(1)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def genericSearch(problem, dataStruct, usesPQ, heuristic=nullHeuristic):
    start = [problem.getStartState()]   # get start state
    struct = dataStruct                 # get generic data structure

    visited = set()                     # set of all coords already seen
    parents = {}                        # dictionary mapping child states to their parents

    pathToState = {}                    # dictionary mapping coords to the total list of directions taken to get to that coord
    pathToState[start[0]] = []          # initialize with start state, and no actions taken

    print "START: ", start

    from copy import deepcopy
    current = deepcopy(start)       # set current node to root node

    # add root node to generic data structure
    if usesPQ:
        struct.push(current, heuristic(current[0], problem))
    else:
        struct.push(current)

    # while data structure not empty
    while not struct.isEmpty():
        current = struct.pop()  # pop from struct

        # if at goal, end
        if problem.isGoalState(current[0]):
            break
        else:
            # get child states from problem
            children = problem.getSuccessors(current[0])

            # for each child state
            for child in children:
                # if not already seen
                if child[0] not in visited:

                    pathToState[child[0]] = pathToState[current[0]] + [child[1]] # add path of parent plus direction taken to get  to child 

                    # add to data structure
                    if usesPQ:
                        struct.push(child, heuristic(child[0], problem) + problem.getCostOfActions(pathToState[child[0]]))
                    else:
                        struct.push(child)

                    # add to visited and update in parents
                    visited.add(child[0])
                    parents[child] = current

    # if didn't find goal, throw error
    if not problem.isGoalState(current[0]):
        throwNoPathError()

    # trace back and reconstruct path
    path = []
    while not checkTupleEquality(current, start):
        path.insert(0, current[1])
        current = parents[current]

    return path

def depthFirstSearch(problem):
    return genericSearch(problem, util.Stack(), False)

def breadthFirstSearch(problem):
    return genericSearch(problem, util.Queue(), False)

def uniformCostSearch(problem):
    return genericSearch(problem, util.PriorityQueue(), True)

def aStarSearch(problem, heuristic=nullHeuristic):
    return genericSearch(problem, util.PriorityQueue(), True, heuristic)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
