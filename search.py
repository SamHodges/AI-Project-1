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
from game import Directions

# node class to keep track of path
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
    # start at given position
    cur_node = Node(None, problem.getStartState(), None)

    # initialize frontier and explored
    frontier = util.Stack()
    explored = set()

    # push first node to frontier
    frontier.push(cur_node)
    
    # while more nodes are on the frontier, continue
    while(not frontier.isEmpty()):
        # get the next state to look into
        cur_node = frontier.pop()

        # check if current node is the goal state
        if(problem.isGoalState(cur_node.state)):
            # if it is a solution, move back up the tree to get the correct list of movements
            solution = []
            while (cur_node.parent is not None):
                solution.append(cur_node.movement)
                cur_node = cur_node.parent

            # reverse the list of movements so it's from start -> finish
            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])

            # return solution
            return final_solution

        # if not a solution, check if it's been explored
        if cur_node.state not in explored:
            # if not, add it to explored and its children to the frontier
            explored.add(cur_node.state)
            for child in problem.getSuccessors(cur_node.state):
                frontier.push(Node(cur_node, child[0], child[1]))

    # if frontier is empty without a solution, search has failed
    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from game import Directions
    print("starting bfs")

    node = Node(None, problem.getStartState(), None)
    if(problem.isGoalState(node.state)):
        return []
    frontier = util.Stack()
    frontier.push(node)
    explored = set()

    while(not frontier.isEmpty()):
        parentNode = frontier.pop()


        if(problem.isGoalState(parentNode.state)):
            solution = []
            node = parentNode
            while (node.parent is not None):
                solution.append(node.movement)
                print()

                node = node.parent

            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])
            print("SOLUTION HERE: ", final_solution)
            return final_solution

        if parentNode.state not in explored:
            explored.add(parentNode.state)
            for child in problem.getSuccessors(parentNode.state):
                # print("adding to frontier: ", parentNode.state, parentNode.movement, child[0], child[1])
                frontier.push(Node(parentNode, child[0], child[1]))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from game import Directions

    print "Start:", problem.getStartState()
    node = Node(None, problem.getStartState(), None)
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
            solution = []
            node = parentNode

            while (node.parent is not None):
                solution.append(node.movement)
                node = node.parent

            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])
            return final_solution

        if parentNode.state not in explored:
            explored.add(parentNode.state)

            for child in problem.getSuccessors(parentNode.state):
            
                temp_node = Node(parentNode, child[0], child[1])
                next_action_cost = child[2]
                temp_node.setPathCost(parentNode.path_cost + next_action_cost)

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
    from game import Directions

    print "Start:", problem.getStartState()
    node = Node(None, problem.getStartState(), None)
    node.setPathCost(0)
    frontier = util.PriorityQueue()
    frontier.push(node, node.path_cost + heuristic(node.state, problem))
    explored = set()

    while(not frontier.isEmpty()):
        parentNode = frontier.pop()
        # print("FRONTIER: ", frontier.isEmpty())
        # print("Parent node is ", parentNode.state)

        if(problem.isGoalState(parentNode.state)):
            solution = []
            node = parentNode

            while (node.parent is not None):
                solution.append(node.movement)
                node = node.parent

            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])
            return final_solution

        

        # print("Expanding node", parentNode.state, ", already explored: ", explored, "already already explored?", parentNode.state in explored)
        if parentNode.state not in explored:
            explored.add(parentNode.state)

            for child in problem.getSuccessors(parentNode.state):
            
                temp_node = Node(parentNode, child[0], child[1])
                next_action_cost = child[2]
                temp_node.setPathCost(parentNode.path_cost + next_action_cost)

                frontier.push(temp_node, temp_node.path_cost + heuristic(temp_node.state, problem))


    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
