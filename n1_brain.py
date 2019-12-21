#!/usr/bin/env python
#Node 1: The robot's brain
import rospy
import cv_bridge
import numpy as np
import cv2
from numpy import linalg
import n4_hit_human_hand
import n3_change_robot_fingers
import ik_initialize
from baxter_interface import gripper as robot_gripper
from sensor_msgs.msg import Range
from std_msgs.msg import String
from sensor_msgs.msg import (
    Image,
)

class GameSession:
    def __init__(self, gameState):
        self.gameState = gameState
        self.accept_human_hand = False
        self.human_move = None

    class Move:
    	def __init__(self, name, act):
    		self.name = name
    		self.act = act


    class GameState:
        def __init__(self, one_left, one_right, two_left, two_right, whos_turn):
            self.one_left = one_left
            self.one_right = one_right
            self.two_left = two_left
            self.two_right = two_right
            self.whos_turn = whos_turn
            
        def __str__(self):
            n3_change_robot_fingers.project_fingers(self.two_left, self.two_right)
            ret_str = "Human Player: [" + str(self.one_left) + "," + str(self.one_right) + "], "
            ret_str += "Baxter: [" + str(self.two_left) + "," + str(self.two_right) + "]"
            ret_str += "\n It's Player " + str(self.whos_turn) + "'s turn!'\n"
            return ret_str

    def human_hand_callback(self, message):
        print("Callback is being called")
        if self.accept_human_hand:
            self.human_move = message.data
            print("Changed human hand move to: " + self.human_move)

    def get_human_move(self):
        return self.human_move

    def set_human_move(self, move):
        self.human_move = move

    def set_accept_human_hand(self, accept):
        self.accept_human_hand = accept

    def get_accept_human_hand(self):
        return self.accept_human_hand

    def ask_user_move(self, state):
        self.set_accept_human_hand(True)
        r = rospy.Rate(1)
        ask_user_input_again = True
        best_move = None
        print("Asking for user move...")
        
        while ask_user_input_again:
            ask_user_input_again = False
            # print("Make your move: [1,2,3,4,5]")
            # print("[1]: Your left to baxter's left")
            # print("[2]: Your left to baxter's right")
            # print("[3]: Your right to baxter's left")
            # print("[4]: Your right to baxter's right")
            # print("[5]: Redistribute")
            # print("Make your move: [1,2,3]")
            # print("[1]: Attack with your left hand")
            # print("[2]: Attack with your right hand")
            # print("[3]: Redistribute")
            user_input = None
            while user_input == None:
                user_input = self.get_human_move()
                rospy.sleep(1)
            
            if user_input == "1":
                if state.one_left == 0:
                    def move(s):
                        new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                        new_s.two_left = new_s.two_left + new_s.one_right
                        if new_s.two_left >= 5:
                            new_s.two_left = 0
                        new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                        return new_s
                    best_move = self.Move("Hitting my right to your left!", move)
                else:
                    def move(s):
                        new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                        new_s.two_left = new_s.two_left + new_s.one_left
                        if new_s.two_left >= 5:
                            new_s.two_left = 0
                        new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                        return new_s
                    best_move = self.Move("Hitting my left to your left!", move)
            elif user_input == "2":
                def move(s):
                    new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                    new_s.two_right = new_s.two_right + new_s.one_left
                    if new_s.two_right >= 5:
                        new_s.two_right = 0
                    new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                    return new_s
                best_move = self.Move("Hitting my left to your right!", move)
            elif user_input == "3":
                def move(s):
                    new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                    new_s.two_left = new_s.two_left + new_s.one_right
                    if new_s.two_left >= 5:
                        new_s.two_left = 0
                    new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                    return new_s
                best_move = self.Move("Hitting my right to your left!", move)
            elif user_input == "4":
                if state.one_right == 0:
                    def move(s):
                        new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                        new_s.two_right = new_s.two_right + new_s.one_left
                        if new_s.two_right >= 5:
                            new_s.two_right = 0
                        new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                        return new_s
                    best_move = self.Move("Hitting my left to your right!", move)
                else:
                    def move(s):
                        new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                        new_s.two_right = new_s.two_right + new_s.one_right
                        if new_s.two_right >= 5:
                            new_s.two_right = 0
                        new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                        return new_s
                    best_move = self.Move("Hitting my right to your right!", move)
            elif user_input == "5":
                print("Please enter in the new finger distribution: [left, right]")
                print("For example: 2,4")
                user_input = raw_input()
                finger_array = user_input.split(",")
                finger_array[0] = finger_array[0].strip()
                finger_array[1] = finger_array[1].strip()
                if is_illegal_redistribution(state, finger_array[0], finger_array[1]):
                    print("Illegal redistribution. You cannot do a mirroring redistribution!")
                    ask_user_input_again = True
                else:
                    move = make_redistribute(1, finger_array[0], finger_array[1])
                    best_move = self.Move("Redistributing!", move)
            else:
                print("Wrong input. Please select a number from 1-5.")
                ask_user_input_again = True
                r.sleep()        
        self.set_accept_human_hand(False)
        self.set_human_move(None)
        return best_move
            
    def find_best_move(self, s):
        best_move = self.minimax(s, 7)[0]
        if best_move == None:
            print("No possible moves were found")
        return best_move

    def minimax(self, s, depth):
        possibleMoves = self.possible_moves(s)
        if depth <= 0 or len(possibleMoves) == 0:
            return [None, self.evaluate(s)]
        best_move = None
        best_score = 0
        if s.whos_turn == 1:
            best_score = -50000
        else:
            best_score = 50000
        for move in possibleMoves:
            new_state = move.act(s)
            _, score = self.minimax(new_state, depth - 1)
            if s.whos_turn == 1:
                if score > best_score:
                    best_score = score
                    best_move = move
            else:
                if score < best_score:
                    best_score = score
                    best_move = move
        return [best_move, best_score]

    def evaluate(self, s):
        if self.has_won(s, 1):
            return 10000
        elif self.has_won(s, 2):
            return -10000
        
        score = 0
        if s.one_left == 0:
            score -= 500
        elif s.one_right == 0:
            score -= 500
        if s.two_left == 0:
            score += 500
        elif s.two_right == 0:
            score += 500
            
        if s.one_left == 4:
            score -= 200
        elif s.one_right == 4:
            score -= 200
        if s.two_left == 4:
            score += 200
        elif s.two_right == 4:
            score += 200
        
        score = score - (s.one_left + s.one_right - s.two_left - s.two_right) * 10
        return score
         
    def possible_moves(self, state):
        ret_arr = []
        if state.whos_turn == 1:
            def move1(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.two_left = new_s.two_left + new_s.one_left
                if new_s.two_left >= 5:
                    new_s.two_left = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            def move2(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.two_right = new_s.two_right + new_s.one_left
                if new_s.two_right >= 5:
                    new_s.two_right = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            def move3(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.two_left = new_s.two_left + new_s.one_right
                if new_s.two_left >= 5:
                    new_s.two_left = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            def move4(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.two_right = new_s.two_right + new_s.one_right
                if new_s.two_right >= 5:
                    new_s.two_right = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            if state.one_left != 0:
                if state.two_left != 0:
                    ret_arr += [self.Move("Hitting my left to your left!", move1)]
                if state.two_right != 0:
                    ret_arr += [self.Move("Hitting my left to your right!", move2)]
            if state.one_right != 0:
                if state.two_left != 0:
                    ret_arr += [self.Move("Hitting my right to your left!", move3)]
                if state.two_right != 0:
                    ret_arr += [self.Move("Hitting my right to your right!", move4)]
            sum = state.one_left + state.one_right
            for i in range(sum + 1):
                def make_move(val):
                    if val == state.one_left or (sum - val) == state.one_right:
                        return
                    def move(s):
                        new_s = GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                        new_s.one_left = val
                        new_s.one_right = sum - val
                        new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                        return new_s
                    return move
                    m = make_move(i)
                    if m and (m(state).one_right != state.one_left):
                        ret_arr += [self.Move("Redistributing!", m)]
        else:       
            def move1(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.one_left = new_s.one_left + new_s.two_left
                if new_s.one_left >= 5:
                    new_s.one_left = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            def move2(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.one_right = new_s.one_right + new_s.two_left
                if new_s.one_right >= 5:
                    new_s.one_right = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            def move3(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.one_left = new_s.one_left + new_s.two_right
                if new_s.one_left >= 5:
                    new_s.one_left = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            def move4(s):
                new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                new_s.one_right = new_s.one_right + new_s.two_right
                if new_s.one_right >= 5:
                    new_s.one_right = 0
                new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                return new_s
            if state.two_left != 0:
                if state.one_left != 0:
                    ret_arr += [self.Move("Hitting my left to your left!", move1)]
                if state.one_right != 0:
                    ret_arr += [self.Move("Hitting my left to your right!", move2)]
            if state.two_right != 0:
                if state.one_left != 0:
                    ret_arr += [self.Move("Hitting my right to your left!", move3)]
                if state.one_right != 0:
                    ret_arr += [self.Move("Hitting my right to your right!", move4)]
            sum = state.two_left + state.two_right
            for i in range(sum + 1):
                def make_move(val):
                    if val == state.one_left or (sum - val) == state.one_right:
                        return
                    def move(s):
                        new_s = self.GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
                        new_s.two_left = val
                        new_s.two_right = sum - val
                        new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
                        return new_s
                    return move
                m = make_move(i)
                if m and (m(state).two_right != state.two_left):
                    ret_arr += [self.Move("Redistributing!", m)]
        return ret_arr

    def is_illegal_redistribution(state, left, right):
        if state.one_left + state.one_right != left + right:
            return True
        elif state.one_left == right and state.one_right == left:
            return True
        return False
    def make_redistribute(player, left, right):
        def redistribute(s):
            new_s = GameState(s.one_left, s.one_right, s.two_left, s.two_right, s.whos_turn)
            if player == 1:
                new_s.one_left = left
                new_s.one_right = right
            else:
                new_s.two_left = left
                new_s.two_right = right
            new_s.whos_turn = max(1, (new_s.whos_turn + 1) % 3)
            return new_s
        return redistribute

    def game_over(self, s):
        return self.has_won(s, 1) or self.has_won(s, 2)

    def has_won(self, s, player):
        if player == 1:
            return s.two_left == 0 and s.two_right == 0
        else:
            return s.one_left == 0 and s.one_right == 0

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

def main():
    #Initialize node
    rospy.init_node('brain', anonymous=True)
    path = '/home/cc/ee106a/fa19/class/ee106a-ada/ros_workspaces/final_project/src/brain/src/Fingers/l1r1.jpg'
    send_image(path)
    gameState = GameSession.GameState(1,1,1,1,1)
    session = GameSession(gameState)
    rospy.Subscriber('ir_human_hand_throttle', String, session.human_hand_callback)
    
    rate = rospy.Rate(1)
    session.accept_human_hand = False

    
    
    #Initialize hands
    ik_initialize.move_hand_init()


    while not session.game_over(session.gameState):
        print(session.gameState)
        best_move = None
        if session.gameState.whos_turn == 1:
            best_move = session.ask_user_move(session.gameState)
            print("Hello")
            print(best_move.name)
        else:
            best_move = session.find_best_move(session.gameState)
            print("Baxter is making his move: " + best_move.name)
            if best_move.name == "Hitting my right to your left!":
            	n4_hit_human_hand.move_hand('right', 'left')
            elif best_move.name == "Hitting my right to your right!":
                n4_hit_human_hand.move_hand('right', 'right')
            elif best_move.name == "Hitting my left to your right!":
                n4_hit_human_hand.move_hand('left', 'right')
            elif best_move.name == "Hitting my left to your left!":
                n4_hit_human_hand.move_hand('left', 'left')        
            rospy.sleep(5)
        if best_move is None:
            raise ValueError

        print("Went out of ask user move")

        session.gameState = best_move.act(session.gameState)
    if session.has_won(session.gameState, 1):
        print("Human has won!")
        path = '/home/cc/ee106a/fa19/class/ee106a-ada/ros_workspaces/final_project/src/brain/src/Fingers/frown.jpeg'
        send_image(path)
    else:
        print("Baxter has won!")
        path = '/home/cc/ee106a/fa19/class/ee106a-ada/ros_workspaces/final_project/src/brain/src/Fingers/smile.jpeg'
        send_image(path)
    

if __name__ == "__main__":
	main()