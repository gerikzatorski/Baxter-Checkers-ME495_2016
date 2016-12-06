#!/usr/bin/env python
"""
This contains many classes to model a checkers (draughts) game on a 6x6 board
"""

import sys

class CheckersGame:
    """ A Checkers Game (The Controller) """
    def __init__(self):
        self.mode = None
        self.board = CheckersBoard() # the model
        self.view = CheckersView() # the view

        # TODO: need to associate the viewer with the model (maybe in their class code as well)
        self.white = Player("human")
        self.red = Player("baxter")
        self.state = GameState.IDLE

    def StartGame(self):
        # Get board game ready and 
        self.board.InitBoard()
        self.state = GameState.WHITE_TURN
        while not self.state == GameState.GAME_OVER:
            self.UpdateView()            
            if self.state == GameState.WHITE_TURN: # human's turn
                human_move = self.HumanMoveInput()
                # TODO: determine if move is legit
                
                self.state = GameState.RED_TURN
            if self.state == GameState.RED_TURN: # Baxter's turn
                # Assuming Baxter's move is legit
                self.state = GameState.WHITE_TURN
        
    def HumanMoveInput(self):
        while True:
            move = raw_input("Your Move.")
            # is input a tile?
            if self.board.tiles.get(move) is None:
                print "That is not a valid tile. Try again"
                continue
            # is the move legit?

            # TODO: more error checking
            else:
                break
        return move
            
        return move

    def UpdateView(self):
        """ Passes board model to view """
        self.view.PrintBoard(self.board)

class GameState:
    IDLE, WHITE_TURN, RED_TURN, GAME_OVER = range(4)

class CheckersBoard:
    """ A Checkers Board (The Model) """
    def __init__(self):
        self.rows = ['A','B','C','D','E','F']
        self.cols = ['1','2','3','4','5','6']
        self.tiles = {'A1': Tile('A1'), 'A2': Tile('A2'), 'A3': Tile('A3'), 'A4': Tile('A4'), 'A5': Tile('A5'), 'A6': Tile('A6'), 'A7': Tile('A7'), 'A8': Tile('A8'),
                      'B1': Tile('B1'), 'B2': Tile('B2'), 'B3': Tile('B3'), 'B4': Tile('B4'), 'B5': Tile('B5'), 'B6': Tile('B6'), 'B7': Tile('B7'), 'B8': Tile('B8'),
                      'C1': Tile('C1'), 'C2': Tile('C2'), 'C3': Tile('C3'), 'C4': Tile('C4'), 'C5': Tile('C5'), 'C6': Tile('C6'), 'C7': Tile('C7'), 'C8': Tile('C8'),
                      'D1': Tile('D1'), 'D2': Tile('D2'), 'D3': Tile('D3'), 'D4': Tile('D4'), 'D5': Tile('D5'), 'D6': Tile('D6'), 'D7': Tile('D7'), 'D8': Tile('D8'),
                      'E1': Tile('E1'), 'E2': Tile('E2'), 'E3': Tile('E3'), 'E4': Tile('E4'), 'E5': Tile('E5'), 'E6': Tile('E6'), 'E7': Tile('E7'), 'E8': Tile('E8'),
                      'F1': Tile('F1'), 'F2': Tile('F2'), 'F3': Tile('F3'), 'F4': Tile('F4'), 'F5': Tile('F5'), 'F6': Tile('F6'), 'F7': Tile('F7'), 'F8': Tile('F8')}
        self.ClearBoard()

    def __str__(self):
        """ Returns a string representation of what the tiles contain """
        s = ""
        for i in self.rows:
            if s != "": s = s + "\n"
            for j in self.cols:
                key = i + j
                s = s + str(self.tiles.get(key)) + " "
        return s

    # 3 "public" methods
    def AcceptRequest(self):
        """ Accepts a a request to make a move by a player """
        return "TODO"

    def ClearBoard(self):
        """ Clears the board by setting all tiles to empty """
        for key, val in self.tiles.iteritems():
            self.tiles[key].state = TileState.EMPTY

    def InitBoard(self):
        """ Sets the board up for a new game with red on top and white on bottom """
        # Place two red rows on top and two white rows on bottom (this doesn't match Tito's email because baxter will actually be playing from that perspective)
        self.tiles['A1'].state = self.tiles['A3'].state = self.tiles['A5'].state = self.tiles['B2'].state = self.tiles['B4'].state = self.tiles['B6'].state = TileState.RED_PIECE
        self.tiles['E1'].state = self.tiles['E3'].state = self.tiles['E5'].state = self.tiles['F2'].state = self.tiles['F4'].state = self.tiles['F6'].state = TileState.RED_PIECE
        # Invalidate moves to some tiles (this is setup to match the picture and setup of Tito's email where the white square are invalid)
        self.tiles['A2'].state = self.tiles['A4'].state = self.tiles['A6'].state = TileState.INVALID
        self.tiles['B1'].state = self.tiles['B3'].state = self.tiles['B5'].state = TileState.INVALID
        self.tiles['C2'].state = self.tiles['C4'].state = self.tiles['C6'].state = TileState.INVALID
        self.tiles['D1'].state = self.tiles['D3'].state = self.tiles['D5'].state = TileState.INVALID
        self.tiles['E2'].state = self.tiles['E4'].state = self.tiles['E6'].state = TileState.INVALID
        self.tiles['F1'].state = self.tiles['F3'].state = self.tiles['F5'].state = TileState.INVALID

    def CurrentPlayer(self):
        """ What player it represents """
        return "TODO"

    def IsOpponent(start, end):
        """ Checks if pieces at locations 1 and 2 are opponents. If either location is unoccupied, returns false. """
        if (self.tiles[start].IsOccupied() or self.tiles[end].IsOccupied()): return False
        # TODO: house cleaning
        color1 = color2 = None
        if (self.tiles[start].state == TileState.RED_PIECE or self.tiles[start].state == TilesState.RED_KING): color1 = 0
        if (self.tiles[start].state == TileState.WHITE_PIECE or self.tiles[start].state == TilesState.WHITE_KING): color1 = 1
        if (self.tiles[end].state == TileState.RED_PIECE or self.tiles[end].state == TilesState.RED_KING): color2 = 0
        if (self.tiles[end].state == TileState.WHITE_PIECE or self.tiles[end].state == TilesState.WHITE_KING): color2 = 0
        return (color1 != color2)

    def ValidMove(start, end):
        start = self.tiles[start]
        end = self.tiles[end]
        """ Returns True if move is valid, however does not consider jumps. """
        # TODO: do we want to replace tile checking logic from HumanMoveInput
        # TODO: can only move your own tiles (since we are only checking humans moves...)
        if self.tiles[start].GetColor == PlayerColor.RED:
            print "You can only move white pieces!"
            return False
        # end must be a valid tile
        if self.tiles[end].IsValidLocation:
            print "That tile is not a valid move tile (lava!)"
            return False
        # end must be unoccupied
        if self.tiles[end].IsOccupied:
            print "That tile is occupied!"
            return False
        # only kings can move backwards
        # assume baxter makes correct moves
        if self.tiles[start].state == TileState.WHITE_PIECE:
            if self.tiles[start] > self.tiles[end]:
                print "That piece can only move up the board"
                return False
        # all pieces can only move 1 row
        if (start.col == 'A' and end.col != 'B'):
            print "Cannot move more than 1 row"
            return False
        if (start.col != 'B' and (end.col != 'A' or end.col != 'C')):
            print "Cannot move more than 1 row"
            return False
        if (start.col != 'C' and (end.col != 'B' or end.col != 'D')):
            print "Cannot move more than 1 row"
            return False
        if (start.col != 'D' and (end.col != 'C' or end.col != 'E')):
            print "Cannot move more than 1 row"
            return False
        if (start.col != 'E' and (end.col != 'D' or end.col != 'F')):
            print "Cannot move more than 1 row"
            return False
        if (start.col != 'F' and end.col != 'E'):
            print "Cannot move more than 1 row"
            return False
        # all pieces must move 1 column TODO: better way to do this
        if (abs(int(start.col) - int(end.col)) != 1):
            print "Cannot move more than 1 column"
            return False
        # Return True if it passes all those tests
        return True

    def ValidJump(start, end):
        """ TODO """
        return False # Return True if the move is a jump over an opponent

    def Move(start, end):
        """ TODO """
        # TODO: capture pieces here
        jumped = ValidJump(start, end)
        if (ValidMove(start, end) or jumped):
            player = self.tiles[start].GetColor == 
        return 0 # Return True if the move has been made
        
    def GameOver():
        """ TODO """
        return False

class Tile:
    """ A Tile on a Checkers Board """
    def __init__(self, location, state = None):
        self.row = location[0]
        self.col = location[1]
        if state is None:
            self.state = TileState.EMPTY
        else:
            self.state = state

    def __str__(self):
        if self.state == TileState.INVALID: return "-"
        return str(self.state)

    def __cmp__(self, other):
        """ Returns True if tile1 is above tile2 """
        # TODO: throw exception if objs aren't of type Tile
        if self.row < this.row: return True
        else: return False

    # TODO: convert this one to __cmp__ version also
    def cmpCol(tile1, tile2):
        """ Returns True if tile1 is left of tile2 """
        if int(tile1.col) > int(tile.col): return True
        else: return False

    def GetColor(self):
        if (self.state == TileState.WHITE_PIECE or self.state == TileState.WHITE_KING): return PlayerColor.WHITE
        if (self.state == TileState.RED_PIECE or self.state == TileState.RED_KING): return PlayerColor.RED
        return "TODO: throw error"

    def IsOccupied(self):
        return (self.state != TileState.EMPTY)

    def IsKing(self):
        return (self.state == TileState.WHITE_KING or self.state == TileState.RED_KING)

    def IsMovingSquare(self):
        return (self.state != TileState.INVALID)
    
    def IsValidLocation(location):
        # TODO: check to make sure location is on the board (ex. Z9 would throw an error)
        return (self.state != TileState.INVALID)        
        
class TileState:
    EMPTY, WHITE_PIECE, RED_PIECE, WHITE_KING, RED_KING, INVALID = range(6)
class PlayerColor:
    WHITE, RED = range(2)

# TODO: Multiple view ?interfaces? (ex. terminal view, window program, etc)
class CheckersView:
    """ The View """
    def __init__(self):
        # TODO: implement different types of views
        self.version = None

    @staticmethod
    def PrintBoard(board):
        print board

### Not Used Currently ###
class Player:
    def __init__(self, name):
        self.name = str(name)

    def MakeMove(self):
        return "TODO"

######################################### Testing #########################################
def main(argv):
    # Fire up a game
    game = CheckersGame()
    game.StartGame()

if __name__ == '__main__':
    main(sys.argv[1:])



######################################### Old Code #########################################
'''
class CheckersBoard:
    """ A Checkers Board """
    def __init__(self):
        self.tiles = [[0 for x in range(6)] for y in range(6)] # might want to setup dict so you can have A1, B4, D6,...
        #TODO: initialize pieces on the board

    def __str__(self):
        s = ""
        for i in range(6):
            s = s + "\n"
            for j in range(6):
                s = s + str(self.tiles[i-1][j]) + " "
        return s
'''
