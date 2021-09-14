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

class Node:
    def __init__(self, parent, state, movement):
        self.parent = parent
        self.state = state
        self.movement = movement
        path_cost = 0

    def setPathCost(self, path_cost):
        self.path_cost = path_cost



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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    from game import Directions

    print "Start:", problem.getStartState()
    node = Node(None, problem.getStartState(), None)
    if(problem.isGoalState(node.state)):
        return []
    frontier = util.Stack()
    frontier.push(node)
    explored = set()

    while(not frontier.isEmpty()):
        parentNode = frontier.pop()
        # print("At node: ", parentNode.state, parentNode.movement)
        # print("FRONTIER: ", frontier.isEmpty())
        # print("Parent node is ", parentNode.state)
        explored.add(parentNode.state)
        for child in problem.getSuccessors(parentNode.state):
            temp_frontier = []
            in_frontier = False

            while(not frontier.isEmpty()):
                cur_frontier = frontier.pop()
                temp_frontier.append(cur_frontier)
                # print(temp_frontier)
                if cur_frontier.state == child[0]:
                    in_frontier = True
                    break
                

            for i in range(len(temp_frontier)):
                frontier.push(temp_frontier[-(i+1)])

            if (child[0] not in explored): #and not in_frontier):
                # print("Looking at child ", child[0])
                # print("Not in ", explored, " or ", temp_frontier)
                if(problem.isGoalState(child[0])):
                    # print("FOUND SOLUTION")
                    solution = []
                    node = Node(parentNode, child[0], child[1])
                    while (node.parent is not None):
                        solution.append(node.movement)
                        node = node.parent

                    final_solution = []
                    for i in range(len(solution)):
                        final_solution.append(solution[-(i+1)])
                    print("Final solution: ", final_solution)
                    return final_solution

                # print("adding to frontier: ", parentNode.state, parentNode.movement, child[0], child[1])
                frontier.push(Node(parentNode, child[0], child[1]))

    print("you failed")
    return False
    # print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    # print "Start's successors:", problem.getSuccessors(problem.getStartState())
    # util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from game import Directions

    print "Start:", problem.getStartState()
    node = Node(None, problem.getStartState())
    if(problem.isGoalState(node.state)):
        return []
    node.setPathCost(0)
    frontier = util.PriorityQueue()
    frontier.push(node, node.path_cost)
    explored = set()

    while(not frontier.isEmpty()):
        parentNode = frontier.pop()
        # print("FRONTIER: ", frontier.isEmpty())
        # print("Parent node is ", parentNode.state)

        if(problem.isGoalState(parentNode.state)):
            print("FOUND SOLUTION")
            solution = []
            node = parentNode

            while (node.parent is not None):
                x_dir = node.parent.state[0] - node.state[0]
                y_dir = node.parent.state[1] - node.state[1]
                next_dir = None
                node = node.parent

                if x_dir > 0:
                    next_dir = Directions.WEST
                if x_dir < 0:
                    next_dir = Directions.EAST
                if y_dir > 0:
                    next_dir = Directions.SOUTH
                if y_dir < 0:
                    next_dir = Directions.NORTH

                solution.append(next_dir)

            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])
            print(final_solution)
            return final_solution

        explored.add(parentNode.state)

        for child in problem.getSuccessors(parentNode.state):
            temp_frontier = []
            in_frontier = False

            temp_node = Node(parentNode, child[0])
            next_action_cost = child[2]
            temp_node.setPathCost(parentNode.path_cost + next_action_cost)

            while(not frontier.isEmpty()):
                cur_frontier = frontier.pop()
                
                if cur_frontier.state == child[0]:
                    if (cur_frontier.path_cost > temp_node.path_cost):
                        temp_frontier.append(temp_node)
                    else:
                        temp_frontier.append(cur_frontier)
                    in_frontier = True
                    break
                else:
                    temp_frontier.append(cur_frontier)
                    
                

            for i in range(len(temp_frontier)):
                frontier.push(temp_frontier[i], temp_frontier[i].path_cost)

            

            if (child[0] not in explored and not in_frontier):
                # print("Looking at child ", child[0])
                # print("Not in ", explored, " or ", temp_frontier)
                frontier.push(temp_node, temp_node.path_cost)


    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
