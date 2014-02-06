# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]
def genericSearch(problem, fringelist, cost):
    """
    A generic search for graphs.
    Uses fringelist as the structure to contain the fringe.
    Cost is a boolean determining whether there is a uniform cost involved. 

    """
    closed = [problem.getStartState()] # the set containing the nodes that we have seen so far
    fringe = fringelist # the fringe containing the paths we have traversed
    #initialize the fringe by appending array paths of the first successors from start
    for state in problem.getSuccessors(problem.getStartState()):
        fringe.push([state], state[2]) if cost else fringe.push([state])
    # main loop    
    while not fringe.isEmpty():
        path = fringe.pop() # remove path from fringe
        lastNode = path[-1][0] # the last traversed node of this path
        if problem.isGoalState(lastNode): #if the last node in the path is a goal node
            #print("Aw yeah we've climbed this mountain.")
            return [x[1] for x in path] # we return the list of actions taken to get to this state
        if lastNode not in closed: # if the node has not been seen 
            closed.append(lastNode) # add the path to the list of seen nodes
            for successor in problem.getSuccessors(lastNode): # iterate through the successors
                if successor[0] not in closed: # check that we do not process an already seen node
                    if not cost:
                        fringe.push(path + [successor]) # push the path including the successor onto the fringe
                    else: 
                        totalCost = 0 # find the cost up to this point
                        for x in path:
                            totalCost += x[2]
                        fringe.push(path + [successor], totalCost + successor[2])

    return []

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    fringe = util.Stack()
    return genericSearch(problem, fringe, False)
    

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    fringe = util.Queue() 
    return genericSearch(problem, fringe, False)

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    fringe = util.PriorityQueue()
    return genericSearch(problem, fringe, True)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic): # defaults to null
    "Search the node that has the lowest combined cost and heuristic first."
    closed = [problem.getStartState()]
    fringe = util.PriorityQueue()
    for state in problem.getSuccessors(problem.getStartState()):
        fringe.push([state], state[2] + heuristic(state[0], problem))  
    while not fringe.isEmpty():
        path = fringe.pop()
        lastNode = path[-1][0]
        if problem.isGoalState(lastNode):
            return [x[1] for x in path]
        if lastNode not in closed: 
            closed.append(lastNode)
            for successor in problem.getSuccessors(lastNode):
                if successor[0] not in closed:
                    totalCost = 0 
                    for x in path:
                        totalCost += x[2]
                    fringe.push(path + [successor], totalCost + successor[2] + heuristic(successor[0], problem))

    return []



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
