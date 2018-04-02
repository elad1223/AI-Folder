from board import Board
from search import SearchProblem, ucs, astar
import util


class BlokusFillProblem(SearchProblem):
    """
    A one-player Blokus game as a search problem.
    This problem is implemented for you. You should NOT change it!
    """

    def __init__(self, board_w, board_h, piece_list, starting_point=(0, 0)):
        self.board = Board(board_w, board_h, 1, piece_list, starting_point)
        self.expanded = 0

    def get_start_state(self):
        """
        Returns the start state for the search problem
        """
        return self.board

    def is_goal_state(self, state):
        """
        state: Search state
        Returns True if and only if the state is a valid goal state
        """
        return not any(state.pieces[0])

    def get_successors(self, state):
        """
        state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        # Note that for the search problem, there is only one player - #0
        self.expanded = self.expanded + 1
        return [(state.do_move(0, move), move, 1) for move in state.get_legal_moves(0)]

    def get_cost_of_actions(self, actions):
        """
        actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        return len(actions)



#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################
class BlokusCornersProblem(SearchProblem):
    def __init__(self, board_w, board_h, piece_list, starting_point=(0, 0)):
        self.expanded = 0
        self.boardW =board_w
        self.boardH =board_h
        self.board = Board(board_w, board_h, 1, piece_list, starting_point)

    def get_start_state(self):
        """
        Returns the start state for the search problem
        """
        return self.board

    def is_goal_state(self, boardState):

        return (boardState.state[0,0] > -1) \
               and (boardState.state[ -1, 0] > -1)\
               and (boardState.state[0,-1] > -1) \
                and (boardState.state[-1,-1] > -1)

    def get_successors(self, state):
        """
        state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        # Note that for the search problem, there is only one player - #0
        self.expanded = self.expanded + 1
        return [(state.do_move(0, move), move, 1) for move in state.get_legal_moves(0)]

    def get_cost_of_actions(self, actions):
        """
        actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        if actions is None:
            return 99999
        cost=0
        for action in actions:
            cost+=len(action.orientation)
        return cost


def blokus_corners_heuristic(BoardState, problem):
    """
    Your heuristic for the BlokusCornersProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come up
    with an admissible heuristic; almost all admissible heuristics will be consistent
    as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the other hand,
    inadmissible or inconsistent heuristics may find optimal solutions, so be careful.
    """
    sum=0
    legals=list()
    for x in range (problem.boardH):
        for y in range(problem.boardW):
            if BoardState.connected[0,y,x] and BoardState.state[y,x] == -1:
                legals.append([x,y])
    if not BoardState.state[0,0]>-1:
        sum+=NewYork(0,0,legals)
    if not BoardState.state[-1,0]>-1:
        sum+=NewYork(problem.boardW-1,0,legals)
    if not BoardState.state[0, -1] > -1:
        sum+=NewYork(0,problem.boardH-1,legals)
    if not BoardState.state[-1, -1] > -1:
        sum += NewYork(problem.boardW - 1, problem.boardH - 1, legals)
    return sum

def NewYork(x,y,options):
    min=1000
    for o in options:
        currentValue=util.manhattanDistance([x,y],[o[0],o[1]])+1
        if currentValue < min:
            min=currentValue
    return min

class BlokusCoverProblem(SearchProblem):
    def __init__(self, board_w, board_h, piece_list, starting_point=(0, 0), targets=[(0, 0)], board=None):
        self.targets = targets.copy()
        self.expanded = 0
        self.startingPoint = starting_point
        self.boardW = board_w
        self.boardH = board_h
        self.pieceList = piece_list
        if board is not None:
            self.board = board
        else:
            self.board = Board(board_w, board_h, 1, piece_list, starting_point)

    def get_start_state(self):
        """
        Returns the start state for the search problem
        """
        return self.board

    def is_goal_state(self, state):
        boo=True
        for target in self.targets:
            boo=(boo and (state.get_position(target[0], target[1]) > -1))

        return boo

    def get_successors(self, state):
        """
        state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        # Note that for the search problem, there is only one player - #0
        self.expanded = self.expanded + 1
        return [(state.do_move(0, move), move, move.piece.get_num_tiles()) for move in state.get_legal_moves(0)]

    def get_cost_of_actions(self, actions):
        """
        actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        if actions is None:
            return 99999
        cost=0
        for action in actions:
            cost+=len(action.orientation)
        return cost


def blokus_cover_heuristic(BoardState, problem):
    sum=0
    legals=list()
    for x in range (problem.boardH):
        for y in range(problem.boardW):
            if BoardState.connected[0,y,x] and BoardState.state[y,x] == -1:
                legals.append([x,y])
    for target in problem.targets:
        if not BoardState.get_position(target[0], target[1]) > -1:
            sum += NewYork(target[0], target[1], legals)

    return sum


class ClosestLocationSearch:
    """
    In this problem you have to cover all given positions on the board,
    but the objective is speed, not optimality.
    """

    def __init__(self, board_w, board_h, piece_list, starting_point=(0, 0), targets=(0, 0)):
        self.expanded = 0
        self.targets = targets.copy()
        self.startingPoint = starting_point
        self.boardW = board_w
        self.boardH = board_h
        self.pieceList = piece_list
        self.board = Board(board_w, board_h, 1, piece_list, starting_point)

    def get_start_state(self):
        """
        Returns the start state for the search problem
        """
        return self.board

    def solve(self):
        """
        This method should return a sequence of actions that covers all target locations on the board.
        This time we trade optimality for speed.
        Therefore, your agent should try and cover one target location at a time. Each time, aiming for the closest uncovered location.
        You may define helpful functions as you wish.

        Probably a good way to start, would be something like this --

        current_state = self.board.__copy__()
        backtrace = []

        while ....

            actions = set of actions that covers the closets uncovered target location
            add actions to backtrace

        return backtrace
        """
        target = self.closestPoint([self.startingPoint])
        path = []
        while target is not None:
            coverProblem = BlokusCoverProblem(self.boardW, self.boardH, self.pieceList, self.startingPoint, [target], self.board)
            actions = astar(coverProblem, blokus_cover_heuristic)
            for action in actions:
                path.append(action)
                self.board.add_move(0, action)
            self.pieceList = self.board.piece_list
            legals = list()
            for x in range(self.boardH):
                for y in range(self.boardW):
                    if self.board.connected[0, y, x] and self.board.state[y, x] == -1:
                        legals.append([x, y])
            target = self.closestPoint(legals)
        return path


    def closestPoint(self,legals):
        minDistance = 1000
        closestPoint = None
        for target in self.targets:
            if not self.board.get_position(target[0], target[1]) > -1:
                distance = NewYork(target[0], target[1], legals)
                if distance < minDistance:
                    minDistance = distance
                    closestPoint = (target[0], target[1])

        return closestPoint

class MiniContestSearch :
    """
    Implement your contest entry here
    """

    def __init__(self, board_w, board_h, piece_list, starting_point=(0, 0), targets=(0, 0)):
        self.targets = targets.copy()
        "*** YOUR CODE HERE ***"

    def get_start_state(self):
        """
        Returns the start state for the search problem
        """
        return self.board

    def solve(self):
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

