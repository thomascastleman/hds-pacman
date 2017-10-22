
# PSEUDOCODE FOR GENERIC SEARCH OPTION

# given Problem problem, some arbitrary data structure dataStruct, optional heuristic function, and boolean usesPQ
def generic_search(problem, dataStruct, heuristic=None, usesPQ, costFunction):

    # get start state

    # dataStruct (given as argument)
    	# stack for DFS
    	# queue for BFS
    	# PQ for UCS
    	# PQ for A*

    # set of (x, y) coords already seen
    # dictionary of children to parent

	# init current at start state

    # add start to dataStruct:
    	# if usesPQ, add with a cost of costFunction called with given heuristic
    	# otherwise, just add w/ single argument

    # while struct not empty

        # pop from struct

        # if at goal state, end
        # else
            # get all successors

			# for all children
                # if not visited
                    # update necessary structures

    # if no path found, throw error

    # trace back and reconstruct path
    # return path

    pass


def UCS_Cost(problem, position, heuristic=None):
	return problem.costFn(position)



def aStarCost(problem, position, heuristic=nullHeuristic):
	return UCS_Cost(problem, position) + heuristic(position, problem)
