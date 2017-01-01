#!/usr/bin/env python

"""
This contains many classes to model a checkers (draughts) game on a 6x6 board

MUST TODO:
require humans to jump

General TODO:
change representation of moves (ex, tuples, separate class, etc)
map tiles to coordinates (ex. A1 to (1.5, 1.5))
"""

import sys
import rospy
from termcolor import colored
from std_msgs.msg import String

# Enum like structures
class GameState:
    IDLE, WHITE_TURN, RED_TURN, GAME_OVER = range(4)
class TileState:
    EMPTY, WHITE_PIECE, RED_PIECE, WHITE_KING, RED_KING, INVALID = range(6)
class PlayerColor:
    WHITE, RED = range(2)
class Columns:
    A, B, C, D, E, F = range(6)

class CheckersGame:
    """ A Checkers Game (The Controller) """
    def __init__(self):
        self.mode = None
        self.board = CheckersBoard() # the model
        self.view = CheckersView() # the view
        # TODO optional: need to associate the viewer with the model (maybe in their class code as well)
        # TODO optional: should I use players?
        # self.white = Player("human")
        #self.red = Player("baxter")
        self.state = GameState.IDLE
        rospy.init_node('checkers_engine')
        self.move_pub = rospy.Publisher('/relay', String, queue_size=10)
        
        '''
        rospy.Subscriber('keys', String, key_pub_rate.keys_callback)
        '''

    def StartGame(self):
        # Get board game ready and 
        self.board.InitBoard()
        self.state = GameState.WHITE_TURN
        while not self.state == GameState.GAME_OVER:
            self.UpdateView()            
            if self.state == GameState.WHITE_TURN: # human's turn
                human_move = self.HumanMoveInput()
                jumped = self.board.Move(PlayerColor.WHITE, human_move[0], human_move[1])
                #if jumped is not None:
                    #print "You took Baxter's piece from {0}! You must jump again if available.".format(jumped)
                    #continue
                self.state = GameState.RED_TURN
            if self.state == GameState.RED_TURN: # Baxter's turn
                baxter_move, dead_man = self.BaxterMove() # Assuming Baxter's move is legit
                #print(self.BaxterMove())
                print "Baxter's Move is {0}".format(baxter_move)
                self.board.Move(PlayerColor.RED, baxter_move[0], baxter_move[1])
                move_cmd = baxter_move[0][0] + ' ' + baxter_move[0][1] + ' ' + baxter_move[1][0] + ' ' + baxter_move[1][1]
                self.move_pub.publish(move_cmd)
                print dead_man
                if dead_man is not None:
                    print "Baxter took your piece from {0}".format(dead_man)
                    move_cmd = dead_man[0] + ' ' + dead_man[1] + ' Z 9'
                
                self.state = GameState.WHITE_TURN

    def OnBoard(self, start, end):
        #if self.board.tiles[start] is None:
        if start not in self.board.tiles:
            print "Start tile is not on the board"
            return False
        if end not in self.board.tiles:
            print "End tile is not on the board"
            return False
        return True
        
    def HumanMoveInput(self):
        
        while True:
            move = raw_input("Your Move: ")
            if ' ' not in move:
                print "Incorrect input format."
                continue
            s = move.split()
            start = s[0]
            end = s[1]
            if self.board.tiles.get(start, None) is None:
                print "Start tile is not on the board"
                continue
            if self.board.tiles.get(end, None) is None:
                print "End tile is not on the board"
                continue
            #if self.board.tiles[start].GetColor == PlayerColor.RED:
            #if self.board.tiles.get(start, None).GetColor() == PlayerColor.RED:
            if self.board.tiles.get(start, None) is not None and self.board.tiles.get(start).GetColor == PlayerColor.RED:
                print "You can only move white pieces!"
                continue
            if not self.board.ValidMove(PlayerColor.WHITE, start, end) and not self.board.ValidJump(PlayerColor.WHITE, start, end)[0]:
                # print outputs controlled by function
                continue
            else:
                break
        return (start, end)

    def BaxterMove(self):
        pieces = []
        for key, val in self.board.tiles.iteritems():
            # TODO optional: keep in mind that tiles only update state and not name
            if val.state == TileState.RED_PIECE or val.state == TileState.RED_KING:
                pieces.append(key)
        available_moves = []
        available_jumps = []
        captured = None
        for start in pieces:
            for key, val in self.board.tiles.iteritems():
                end = key
                if self.board.ValidMove(PlayerColor.RED, start, end):
                    available_moves.append((start, end))
                if self.board.ValidJump(PlayerColor.RED, start, end)[0]:
                    available_jumps.append((start, end))
        if len(available_jumps) > 0:
            captured = self.board.ValidJump(PlayerColor.RED, available_jumps[0][0], available_jumps[0][1])[1]
            return available_jumps[0], captured
        else:
            return available_moves[0], None
        
    def UpdateView(self):
        """ Passes board model to view """
        self.view.PrintBoard(self.board)
        #self.view.PrintColor(self.board)
        

class CheckersBoard:
    """ A Checkers Board (The Model) """
    def __init__(self):
        self.rows = ['6','5','4','3','2','1']
        self.cols = ['A','B','C','D','E','F']
        self.tiles = {'A6': Tile('A6'), 'B6': Tile('B6'), 'C6': Tile('C6'), 'D6': Tile('D6'), 'E6': Tile('E6'), 'F6': Tile('F6'),
                      'A5': Tile('A5'), 'B5': Tile('B5'), 'C5': Tile('C5'), 'D5': Tile('D5'), 'E5': Tile('E5'), 'F5': Tile('F5'),
                      'A4': Tile('A4'), 'B4': Tile('B4'), 'C4': Tile('C4'), 'D4': Tile('D4'), 'E4': Tile('E4'), 'F4': Tile('F4'),
                      'A3': Tile('A3'), 'B3': Tile('B3'), 'C3': Tile('C3'), 'D3': Tile('D3'), 'E3': Tile('E3'), 'F3': Tile('F3'),
                      'A2': Tile('A2'), 'B2': Tile('B2'), 'C2': Tile('C2'), 'D2': Tile('D2'), 'E2': Tile('E2'), 'F2': Tile('F2'),
                      'A1': Tile('A1'), 'B1': Tile('B1'), 'C1': Tile('C1'), 'D1': Tile('D1'), 'E1': Tile('E1'), 'F1': Tile('F1')}

        '''
        self.tiles = {'A1': Tile('A1'), 'A2': Tile('A2'), 'A3': Tile('A3'), 'A4': Tile('A4'), 'A5': Tile('A5'), 'A6': Tile('A6'), 'A7': Tile('A7'), 'A8': Tile('A8'),
                      'B1': Tile('B1'), 'B2': Tile('B2'), 'B3': Tile('B3'), 'B4': Tile('B4'), 'B5': Tile('B5'), 'B6': Tile('B6'), 'B7': Tile('B7'), 'B8': Tile('B8'),
                      'C1': Tile('C1'), 'C2': Tile('C2'), 'C3': Tile('C3'), 'C4': Tile('C4'), 'C5': Tile('C5'), 'C6': Tile('C6'), 'C7': Tile('C7'), 'C8': Tile('C8'),
                      'D1': Tile('D1'), 'D2': Tile('D2'), 'D3': Tile('D3'), 'D4': Tile('D4'), 'D5': Tile('D5'), 'D6': Tile('D6'), 'D7': Tile('D7'), 'D8': Tile('D8'),
                      'E1': Tile('E1'), 'E2': Tile('E2'), 'E3': Tile('E3'), 'E4': Tile('E4'), 'E5': Tile('E5'), 'E6': Tile('E6'), 'E7': Tile('E7'), 'E8': Tile('E8'),
                      'F1': Tile('F1'), 'F2': Tile('F2'), 'F3': Tile('F3'), 'F4': Tile('F4'), 'F5': Tile('F5'), 'F6': Tile('F6'), 'F7': Tile('F7'), 'F8': Tile('F8')}
        '''
        self.ClearBoard()

    def __str__(self):
        """ Returns a string representation of what the tiles contain """
        s = ""
        for i in self.rows:
            if s != "": s = s + "\n"
            for j in self.cols:
                key = j + i
                s = s + str(self.tiles.get(key)) + " "
        return s

    def PrintColor(self):
        color = 'blue'
        blah = []
        s = ""
        for i in self.rows:
            if s != "": s = s + "\n"
            for j in self.cols:
                key = j + i
                if self.tiles[key].GetColor() == PlayerColor.RED: color = 'red'
                if self.tiles[key].GetColor() == PlayerColor.WHITE: color = 'white'
                s = s + str(self.tiles.get(key)) + " "
                blah.append[s]
        print blah
        return 0
                
    def ClearBoard(self):
        """ Clears the board by setting all tiles to empty """
        for key, val in self.tiles.iteritems():
            self.tiles[key].state = TileState.EMPTY

    def InitBoard(self):
        """ Sets the board up for a new game with red on top and white on bottom """
        # Place two red rows on top and two white rows on bottom
        self.tiles['A1'].state = self.tiles['C1'].state = self.tiles['E1'].state = self.tiles['B2'].state = self.tiles['D2'].state = self.tiles['F2'].state = TileState.WHITE_PIECE
        self.tiles['B6'].state = self.tiles['D6'].state = self.tiles['F6'].state = self.tiles['A5'].state = self.tiles['C5'].state = self.tiles['E5'].state = TileState.RED_PIECE
        # Invalidate moves to some tiles
        self.tiles['A2'].state = self.tiles['A4'].state = self.tiles['A6'].state = TileState.INVALID
        self.tiles['B1'].state = self.tiles['B3'].state = self.tiles['B5'].state = TileState.INVALID
        self.tiles['C2'].state = self.tiles['C4'].state = self.tiles['C6'].state = TileState.INVALID
        self.tiles['D1'].state = self.tiles['D3'].state = self.tiles['D5'].state = TileState.INVALID
        self.tiles['E2'].state = self.tiles['E4'].state = self.tiles['E6'].state = TileState.INVALID
        self.tiles['F1'].state = self.tiles['F3'].state = self.tiles['F5'].state = TileState.INVALID

    def CurrentPlayer(self, start):
        """ What player a move represents. Used to determine tile color of player start. """
        if self.tiles[start].state == TileState.RED_PIECE or self.tiles[start].state == TileState.RED_KING:
            return PlayerColor.RED
        if self.tiles[start].state == TileState.WHITE_PIECE or self.tiles[start].state == TileState.WHITE_KING:
            return PlayerColor.WHITE
        else: return False # error checking on error checking

    def IsOpponent(self, start, end):
        """ Checks if pieces at locations 1 and 2 are opponents. If either location is unoccupied, returns false. """
        if (self.tiles[start].IsOccupied() or self.tiles[end].IsOccupied()): return False
        # TODO: house cleaning
        color1 = color2 = None
        if (self.tiles[start].state == TileState.RED_PIECE or self.tiles[start].state == TilesState.RED_KING): color1 = 0
        if (self.tiles[start].state == TileState.WHITE_PIECE or self.tiles[start].state == TilesState.WHITE_KING): color1 = 1
        if (self.tiles[end].state == TileState.RED_PIECE or self.tiles[end].state == TilesState.RED_KING): color2 = 0
        if (self.tiles[end].state == TileState.WHITE_PIECE or self.tiles[end].state == TilesState.WHITE_KING): color2 = 0
        return (color1 != color2)

    def ValidMove(self, color, start, end):
        """
        Assume: start and end are pieces on the board
        Returns True if move is valid, however does not consider jumps.
        """
        # TODO: remove repeated code in ValidJump
        human = (color == PlayerColor.WHITE)
        if start == end:
            if human: print "Origin and destination cannot be the same."
            return False
        start = self.tiles[start]
        end = self.tiles[end]
        if not start.IsValidTile():
            if human: print "Start is not a valid move tile (lava!)"
            return False
        # end must be a valid tile
        if not end.IsValidTile():
            if human: print "Destination is not a valid move tile (lava!)"
            return False
        # end must be unoccupied
        if end.IsOccupied():
            if human: print "That tile is occupied!"
            return False
        # only kings can move backwards
        if start.state == TileState.WHITE_PIECE:
            if start.row > end.row:
                if human: print "That piece can only move up the board"
                return False
        if start.state == TileState.RED_PIECE:
            if start.row < end.row:
                if human: print "That piece can only move down the board"
                return False
        # End of repeated code

        # all pieces can only  move 1 row
        if (abs(int(start.row) - int(end.row)) != 1):
            if human: print "Cannot move more than 1 column"
            return False
        # all pieces can only move 1 column
        # TODO: remove repeated code
        if (start.col == 'A' and not end.col == 'B'):
            if human: print "Cannot move more than 1 col"
            return False
        if (start.col == 'B' and not (end.col == 'A' or end.col == 'C')):
            if human: print "Cannot move more than 1 col"
            return False
        if (start.col == 'C' and not (end.col == 'B' or end.col == 'D')):
            if human: print "Cannot move more than 1 col"
            return False
        if (start.col == 'D' and not (end.col == 'C' or end.col == 'E')):
            if human: print "Cannot move more than 1 col"
            return False
        if (start.col == 'E' and not (end.col == 'D' or end.col == 'F')):
            if human: print "Cannot move more than 1 col"
            return False
        if (start.col == 'F' and not end.col == 'E'):
            if human: print "Cannot move more than 1 col"
            return False
        # Return True if it passes all those tests
        return True

    def ValidJump(self, color, start, end):
        """ Returns False if not valid and tuple of (true, jumped tile key) if valid """
        # TODO: remove repeated code from ValidMove
        human = (color == PlayerColor.WHITE)
        if start == end:
            if human: print "Origin and destination cannot be the same."
            return (False, None)
        start = self.tiles[start]
        end = self.tiles[end]
        if not start.IsValidTile():
            if human: print "Start is not a valid move tile (lava!)"
            return (False, None)
        # end must be a valid tile
        if not end.IsValidTile():
            if human: print "Destination is not a valid move tile (lava!)"
            return (False, None)
        # end must be unoccupied
        if end.IsOccupied():
            if human: print "That tile is occupied!"
            return (False, None)
        # only kings can move backwards
        if start.state == TileState.WHITE_PIECE:
            if start.row >end.row:
                if human: print "That piece can only move up the board"
                return (False, None)
        if start.state == TileState.RED_PIECE:
            if start.row < end.row:
                if human: print "That piece can only move down the board"
                return (False, None)
        # End of repeated code

        columnstart = None
        if start.col == 'A': columnstart = Columns.A
        if start.col == 'B': columnstart = Columns.B
        if start.col == 'C': columnstart = Columns.C
        if start.col == 'D': columnstart = Columns.D
        if start.col == 'E': columnstart = Columns.E
        if start.col == 'F': columnstart = Columns.F

        columnend = None
        if end.col == 'A': columnend = Columns.A
        if end.col == 'B': columnend = Columns.B
        if end.col == 'C': columnend = Columns.C
        if end.col == 'D': columnend = Columns.D
        if end.col == 'E': columnend = Columns.E
        if end.col == 'F': columnend = Columns.F

        # jump has to move 2 rows
        if (abs(int(start.row) - int(end.row)) != 2):
            if human: print "Jumps must be 2 rows"
            return (False, None)
        # jump has to move 2 columns
        if (start.col == 'A' and not end.col == 'C'):
            if human: print "Jumps must be 2 columns"
            return (False, None)
        if (start.col == 'B' and not end.col == 'D'):
            if human: print "Jumps must be 2 columns"
            return (False, None)
        if (start.col == 'C' and not (end.col == 'A' or end.col == 'E')):
            if human: print "Jumps must be 2 columns"
            return (False, None)
        if (start.col == 'D' and not (end.col == 'B' or end.col == 'F')):
            if human: print "Jumps must be 2 columns"
            return (False, None)
        if (start.col == 'E' and not end.col == 'C'):
            if human: print "Jumps must be 2 columns"
            return (False, None)
        if (start.col == 'F' and not end.col == 'D'):
            if human: print "Jumps must be 2 columns"
            return (False, None)

        # is there an opponent tile between?
        mid_col = (columnstart + columnend) / 2
        mid_row = (int(start.row) + int(end.row)) / 2
        mid_row_char = str(mid_row)

        mid_col_char = None
        if mid_col == Columns.A: mid_col_char = 'A'
        if mid_col == Columns.B: mid_col_char = 'B'
        if mid_col == Columns.C: mid_col_char = 'C'
        if mid_col == Columns.D: mid_col_char = 'D'
        if mid_col == Columns.E: mid_col_char = 'E'
        if mid_col == Columns.F: mid_col_char = 'F'

        mid_tile = mid_col_char + mid_row_char

        if (start.GetColor() == PlayerColor.RED and self.tiles[mid_tile].GetColor() != PlayerColor.WHITE): return (False, None)
        if (start.GetColor() == PlayerColor.WHITE and self.tiles[mid_tile].GetColor() != PlayerColor.RED): return (False, None)

        return (True, mid_tile) # Return Tile location if jump is true

    def Move(self, color, start, end):
        """ Returns the tile key of jumped piece (if jumped exists) """
        jumped = self.ValidJump(color, start, end)
        if (self.ValidMove(color, start, end) or jumped[0]):
            # pick up piece
            in_hand = self.tiles[start].state
            self.tiles[start].state = TileState.EMPTY
            # put piece down
            self.tiles[end].state = in_hand
            if jumped[0]:
                self.tiles[jumped[1]].state = TileState.EMPTY
        # do we king the tile?
        if color == PlayerColor.RED and self.tiles.get(end).row == '1':
            print "King that red piece!"
            self.tiles.get(end).state = TileState.RED_KING
        if color == PlayerColor.WHITE and self.tiles.get(end).row == '6':
            print "King that white piece!"
            self.tiles.get(end).state = TileState.WHITE_KING
        return jumped[1] # Return tile key of jumped piece, will be None is not jumped
        
    def GameOver(self):
        """ TODO """
        return False

class Tile:
    """ A Tile on a Checkers Board """
    def __init__(self, location, state = None):
        self.col = location[0]
        self.row = location[1]
        if state is None:
            self.state = TileState.EMPTY
        else:
            self.state = state

    def __str__(self):
        if self.state == TileState.INVALID: return "-"
        return str(self.state)

    def __cmp__(self, other):
        """ Returns True if tile1 is to the right of tile2 """
        # TODO: throw exception if objs aren't of type Tile
        if self.col == 'A': col_value1 = Columns.A
        if self.col == 'B': col_value1 = Columns.B
        if self.col == 'C': col_value1 = Columns.C
        if self.col == 'D': col_value1 = Columns.D
        if self.col == 'E': col_value1 = Columns.E
        if self.col == 'F': col_value1 = Columns.F

        if other.col == 'A': col_value2 = Columns.A
        if other.col == 'B': col_value2 = Columns.B
        if other.col == 'C': col_value2 = Columns.C
        if other.col == 'D': col_value2 = Columns.D
        if other.col == 'E': col_value2 = Columns.E
        if other.col == 'F': col_value2 = Columns.F

        if col_value1 > col_value2: return True
        else: return False

    # TODO: convert this one to __cmp__ version also
    def cmpCol(tile1, tile2):
        """ Returns True if tile1 is left of tile2 """
        if int(tile1.col) > int(tile.col): return True
        else: return False

    def GetColor(self):
        if (self.state == TileState.WHITE_PIECE or self.state == TileState.WHITE_KING): return PlayerColor.WHITE
        if (self.state == TileState.RED_PIECE or self.state == TileState.RED_KING): return PlayerColor.RED
        return -1

    def IsOccupied(self):
        return (self.state != TileState.EMPTY)

    def IsKing(self):
        return (self.state == TileState.WHITE_KING or self.state == TileState.RED_KING)

    def IsValidTile(self):
        if self.state == TileState.INVALID: return False
        else: return True
        #return (self.state != TileState.INVALID)
        
# TODO: Multiple view ?interfaces? (ex. terminal view, window program, etc)
class CheckersView:
    """ The View """
    def __init__(self):
        # TODO: implement different types of views
        self.version = None

    @staticmethod
    def PrintBoard(board):
        print board

    @staticmethod
    def PrintColor(board):
        board.PrintColor()

### Not Used Currently ###
class Player:
    def __init__(self, name):
        self.name = str(name)

    def MakeMove(self):
        return "TODO"

######################################### Testing #########################################
def main(argv):
    while not rospy.is_shutdown():
        game = CheckersGame()
        game.StartGame()

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass
    #except KeyboardInterrupt:
