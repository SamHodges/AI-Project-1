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

    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # initialize root of tree at starting state
    cur_node = Node(None, problem.getStartState(), None)
    cur_node.setPathCost(0)

    # initialize empty frontier and explored sets
    frontier = util.PriorityQueue()
    explored = set()

    # push node to frontier, organized by path cost
    frontier.push(cur_node, cur_node.path_cost)

    # while more states in frontier, continue
    while(not frontier.isEmpty()):
        # get next state from frontier
        cur_node = frontier.pop()

        # check if current node is a solution
        if(problem.isGoalState(cur_node.state)):
            # if it is, go back up through tree to get list of actions
            solution = []
            while (cur_node.parent is not None):
                solution.append(cur_node.movement)
                cur_node = cur_node.parent

            # reverse list of actions so it's from start -> finish
            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])

            # return list of actions
            return final_solution

        # check if current state has been explored
        if cur_node.state not in explored:
            # if not, add it to explored
            explored.add(cur_node.state)

            # add children to frontier
            for child in problem.getSuccessors(cur_node.state):
                # create node for current child state
                child_node = Node(cur_node, child[0], child[1])

                # calculate overall path cost by adding next step to parent's path cost
                next_action_cost = child[2]
                child_node.setPathCost(cur_node.path_cost + next_action_cost)

                # push node to frontier ordered by path cost
                frontier.push(child_node, child_node.path_cost)

    # if frontier is empty without solution, return failure
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
    # create tree root at start state
    cur_node = Node(None, problem.getStartState(), None)
    cur_node.setPathCost(0)

    # initialize frontier and explored set
    frontier = util.PriorityQueue()
    explored = set()

    # add root node to frontier
    frontier.push(cur_node, cur_node.path_cost + heuristic(cur_node.state, problem))

    # while frontier is not empty, continue loop
    while(not frontier.isEmpty()):
        # get next node from frontier
        cur_node = frontier.pop()
        
        # check if state is goal state
        if(problem.isGoalState(cur_node.state)):
            # if it is, go back up to root to create list of actions
            solution = []
            while (cur_node.parent is not None):
                solution.append(cur_node.movement)
                cur_node = cur_node.parent

            # reverse actions so they go from start -> finish
            final_solution = []
            for i in range(len(solution)):
                final_solution.append(solution[-(i+1)])

            # return list of actions
            return final_solution

        # check if state is in explored
        if cur_node.state not in explored:
            # if not, add it
            explored.add(cur_node.state)

            # add children to frontier
            for child in problem.getSuccessors(cur_node.state):
                # create node for next state
                child_node = Node(cur_node, child[0], child[1])

                # calculate child path cost by adding cost of next step to parent's path cost
                next_action_cost = child[2]
                child_node.setPathCost(cur_node.path_cost + next_action_cost)

                # push node to frontier ordered by heuristic
                frontier.push(child_node, child_node.path_cost + heuristic(child_node.state, problem))

    # if it doesn't find a solution, return failure
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
