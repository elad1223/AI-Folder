"""
In search.py, you will implement generic search algorithms
"""

import util as util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def get_start_state(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def is_goal_state(self, state):
        """
        state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def get_successors(self, state):
        """
        state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def get_cost_of_actions(self, actions):
        """
        actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()

def depth_first_search(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches
    the goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    return DFS((problem.get_start_state(),None),problem,list(),list())

def DFS(currentTruple,problem,path,visited):
    currentState=currentTruple[0]
    if visited.__len__() >0:
        path.append(currentTruple[1])
    visited.append(currentState)
    if(problem.is_goal_state(currentState)):
        return path
    for states in problem.get_successors(currentState)[::-1]:
        if states[0] not in visited:
            returnPath= DFS(states,problem,path.copy(),visited)
            if returnPath.__len__()>0 :
                return returnPath
    return list()

def breadth_first_search(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    fringe=util.Queue()
    fringe.push(((problem.get_start_state(),None,0),list()))
    visited=list()
    while not fringe.isEmpty():
        currentNode=fringe.pop()
        currentPath=currentNode[1]
        currentTruple=currentNode[0]
        currentState=currentTruple[0]
        if currentState in visited:
            continue
        if currentTruple[1] is not None:
            currentPath.append(currentTruple[1])
        visited.append(currentState)
        if problem.is_goal_state(currentState):
            return currentPath
        for states in problem.get_successors(currentState):
            if problem.is_goal_state(states[0]):
                currentPath.append(states[1])
                return currentPath
            if states[0] not in visited:
                fringe.push((states,currentPath.copy()))
    return list()

def uniform_cost_search(problem):
    """
    Search the node of least total cost first.
    """
    fringe=util.PriorityQueue()
    fringe.push(UNCNode((list(),(problem.get_start_state(),None,0)),0),0)
    winningPath=None
    winnningCost=0
    visited=set()
    while not fringe.isEmpty():
        currentNode=fringe.pop()
        currentCost=currentNode.price
        currentPath=currentNode.item[0]
        currentTruple=currentNode.item[1]
        currentState=currentTruple[0]
        if currentState in visited:
            continue
        if not currentCost==0:
            currentPath.append(currentTruple[1])
        visited.add(currentState)
        for states in problem.get_successors(currentState):
            if problem.is_goal_state(states[0]):
                if (winningPath is None) or (currentCost+states[2]<winnningCost): #if we found a better solution
                    winningPath=currentPath.copy()
                    winningPath.append(states[1])
                    winnningCost=states[2]+currentCost

            elif states[0] not in visited:
                fringe.push(UNCNode((currentPath.copy(),states),currentCost+states[2]),currentCost+states[2])
        if winningPath is not None: #if we pass the score we can return
            #if currentCost>=winnningCost:
                return winningPath

    return list()

class UNCNode:
    def __init__(self,item,price):
        self.price=price
        self.item=item
    def __le__(self, other):
        return util.flipCoin(0.5)

def null_heuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def a_star_search(problem, heuristic=null_heuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    fringe = util.PriorityQueue()
    fringe.push(UNCNode((list(), (problem.get_start_state(), None, 0)), 0), 0)
    visited = set()
    winningPath = None
    winnningCost = 100000
    while not fringe.isEmpty():
        currentNode = fringe.pop()
        currentPath = currentNode.item[0]
        currentTruple = currentNode.item[1]
        currentState = currentTruple[0]
        if currentState in visited:
            continue
        if currentTruple[1] is not None:
            currentPath.append(currentTruple[1])
        visited.add(currentState)
        if winnningCost <= currentNode.price:
            return winningPath
        for states in problem.get_successors(currentState):
            cost = problem.get_cost_of_actions(currentPath) + states[2] + heuristic(states[0], problem)
            if problem.is_goal_state(states[0]):
                if (winningPath is None) or (cost<winnningCost): #if we found a better solution
                    winningPath=currentPath.copy()
                    winningPath.append(states[1])
                    winnningCost=cost
            elif states[0] not in visited:
                cost=problem.get_cost_of_actions(currentPath) + states[2]+ heuristic(states[0],problem)
                fringe.push(UNCNode((currentPath.copy(), states),cost ),cost)
    return list()

# Abbreviations
bfs = breadth_first_search
dfs = depth_first_search
astar = a_star_search
ucs = uniform_cost_search
