# -*- coding: utf-8 -*-
"""
Created on Sat Feb 11 17:14:01 2017

@author: jc4730/JiadaChen/CS4701/HW1
"""

import sys
import math
import time
import resource
from collections import deque
from heapq import *

def bfs(initial):
    initial_state= State(initial)
    goal_state= State(range(len(initial)))
    
    frontier= deque()
    frontier.append(initial_state)
    explored= set()
    frontier_set = set()
    frontier_set.add(initial_state)
    
    mfs = 0
    msd = 0
    
    while not (len(frontier) == 0):
        mfs = max(mfs,len(frontier))
        state= frontier.popleft()
        
        if goal_state == state:
            return frontier,explored,state,msd,mfs
        explored.add(state)
        
        neighbors = state.neighbors()
        if neighbors:
        
            for neighbor in neighbors:
                if (neighbor not in explored) & (neighbor not in frontier_set):
                    frontier.append(neighbor)
                    frontier_set.add(neighbor)
                    msd = max(msd,neighbor.path_cost)
    
    return frontier,explored,None,msd,mfs



def dfs(initial):
    initial_state = State(initial)
    goal_state = State(range(len(initial)))
    
    explored = set()
    frontier = deque()
    frontier_set = set()
    frontier.append(initial_state)
    frontier_set.add(initial_state)
    
    mfs = 0
    msd = 0
    
    while len(frontier) <> 0:
        mfs = max(mfs,len(frontier))
        state = frontier.pop()
        
        if goal_state == state:
            return frontier,explored,state,msd,mfs
        
        explored.add(state)
        neighbors = state.neighbors()
        
        if neighbors:
            neighbors.reverse()

            for neighbor in neighbors:
                if (neighbor not in explored) & (neighbor not in frontier_set):
                    frontier.append(neighbor)
                    frontier_set.add(neighbor)
                    msd = max(msd,neighbor.path_cost)

    return frontier,explored,None,msd,mfs



def ast(initial):
    initial_state = State(initial,heur=h_manhattan(initial))
    goal_state = State(range(len(initial)))
    
    explored = set()
    frontier = []
    frontier_set = set()
    heappush(frontier,initial_state)
    frontier_set.add(initial_state)
    
    mfs = 0
    msd = 0
    
    while len(frontier) <> 0:
        mfs = max(mfs,len(frontier))
        state = heappop(frontier)
        
        if goal_state == state:
            return frontier,explored,state,msd,mfs
        
        explored.add(state)
        
        neighbors = state.neighbors()
        if neighbors:
            for neighbor in neighbors:
                if (neighbor not in explored) & (neighbor not in frontier_set):
                    heappush(frontier,neighbor)
                    frontier_set.add(neighbor)
                    msd = max(msd,neighbor.path_cost)
                elif neighbor in frontier:
                    frontier = decreaseKey(frontier,neighbor)
    
    return frontier,explored,None,msd,mfs



def dlst(initial,limit,exp,msd,mfs):
    initial_state = State(initial,heur=h_manhattan(initial))
    goal_state = State(range(len(initial)))
    
    explored = {}
    frontier = deque()
    frontier_set = set()
    frontier.append(initial_state)
    frontier_set.add(initial_state)
    
    l = float("inf")
    
    while (len(frontier) <> 0):
        mfs = max(mfs,len(frontier))
        state = frontier.pop()
        
        if goal_state == state:
            return state,limit,frontier,exp,msd,mfs
        
        explored[state] = state.path_cost
        exp += 1
        neighbors = state.neighbors()

        if neighbors:
            neighbors.reverse()

            for neighbor in neighbors:
                f_n = neighbor.path_cost + neighbor.heur
                l = min(l,f_n)
                if (f_n <= limit) & (neighbor not in explored) & (neighbor not in frontier_set):
                    #print 'a'
                    frontier.append(neighbor)
                    frontier_set.add(neighbor)
                    msd = max(msd,neighbor.path_cost)
                elif neighbor in explored:
                    if (neighbor.path_cost <= explored[neighbor]):
                        frontier.append(neighbor)
                        msd = max(msd,neighbor.path_cost)
    limit = limit+1 if l <= limit else l
    return None,limit,frontier,exp,msd,mfs


def ida(initial):
    limit = h_manhattan(initial)
    exp = 0
    mfs = 0
    msd = 0
    while True:
        state,l,frontier,exp,msd,mfs = dlst(initial,limit,exp,msd,mfs)
        if state:
            return frontier,exp,state,msd,mfs
        else:
            limit = l

        



def h_manhattan(board):
    n = int(math.sqrt(len(board)))
    manhattan_dist = 0

    for i in range(1,n**2):
        tr = i/n
        tc = i%n
        ind = board.index(i)
        r = ind/n
        c = ind%n
        i_manhattan = abs(tr - r) + abs(tc - c)
        manhattan_dist += i_manhattan

    return manhattan_dist


def decreaseKey(frontier,neighbor):
    update = frontier[frontier.index(neighbor)]
    if neighbor < update:
        update.heur = neighbor.heur
        heapify(frontier)
    return frontier


class State:
    def __init__(self,board,parent=None,action=None,path_cost=0,heur=0):
        # board(int[]): representing tiles within the board
        self.board= board
        self.parent= parent
        self.action= action
        self.path_cost = path_cost
        self.heur = heur
    
    def __eq__(self,other):
        return self.board == other.board
    
    def __hash__(self):
        return hash(str(self.board))
    
    def __cmp__(self,other):
        #return cmp(self.path_cost + self.heur,other.path_cost + other.heur)
        
        udlr = {'Up':1,'Down':2,'Left':3,'Right':4}
        
        if (self.path_cost + self.heur) < (other.path_cost + other.heur):
            return -1
        elif (self.path_cost + self.heur) > (other.path_cost + other.heur):
            return 1
        else:
            return cmp(udlr[self.action],udlr[other.action])
        
        
    def neighbors(self):
        # Given a board state, return all its children states
        # return(State[])
        neighbors= deque()
        
        n = int(math.sqrt(len(self.board)))
        zero= self.board.index(0)
        
        if (zero/n != 0):
            neighbors.append(self.up())
        if (zero/n != n-1):
            neighbors.append(self.down())
        if (zero%n != 0):
            neighbors.append(self.left())
        if (zero%n != n-1):
            neighbors.append(self.right())
        
        return neighbors
        
        
    def up(self):
        future_board= self.board[:]
        n = int(math.sqrt(len(future_board)))
        
        zero = self.board.index(0)
        future_board[zero-n], future_board[zero]= future_board[zero],future_board[zero-n]
        result_state = State(board=future_board,parent=self,action='Up',path_cost=self.path_cost+1,heur=h_manhattan(future_board))
        
        return result_state
    
    def down(self):
        future_board= self.board[:]
        n = int(math.sqrt(len(future_board)))
        
        zero = self.board.index(0)
        future_board[zero+n], future_board[zero]= future_board[zero],future_board[zero+n]
        result_state = State(board=future_board,parent=self,action='Down',path_cost=self.path_cost+1,heur=h_manhattan(future_board))
        
        return result_state
    
    def left(self):
        future_board= self.board[:]
        
        zero = self.board.index(0)
        future_board[zero-1], future_board[zero]= future_board[zero],future_board[zero-1]
        result_state = State(board=future_board,parent=self,action='Left',path_cost=self.path_cost+1,heur=h_manhattan(future_board))
        
        return result_state
    
    def right(self):
        future_board= self.board[:]
        
        zero = self.board.index(0)
        future_board[zero+1], future_board[zero]= future_board[zero],future_board[zero+1]
        result_state = State(board=future_board,parent=self,action='Right',path_cost=self.path_cost+1,heur=h_manhattan(future_board))
        
        return result_state
    
    def get_path(self):
        path_to_goal = []
        while self.parent is not None:
            path_to_goal.append(self.action)
            self = self.parent
        path_to_goal.reverse()
        
        return path_to_goal

    
def main():
    start_time = time.time()
    start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    search = sys.argv[1]
    initial = sys.argv[2]
    
    initial = map(int,initial.split(','))
    
    if search == 'bfs':
        a,b,c,d,e = bfs(initial)
        b = len(b)
    elif search == 'dfs':
        a,b,c,d,e = dfs(initial)
        b = len(b)
    elif search == 'ast':
        a,b,c,d,e = ast(initial)
        b = len(b)
    elif search == 'ida':
        a,b,c,d,e = ida(initial)
    else: 
        return None
    
    path_to_goal = c.get_path()
    end_time = time.time()
    run_time = end_time - start_time
    end_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss  
    run_ram = (end_ram - start_ram)/(1024.*1024.)
    
    textList = []
    textList.append("path_to_goal: " + str(path_to_goal)+"\n")
    textList.append("cost_of_path: " + str(c.path_cost)+"\n")
    textList.append("nodes_expanded: " + str(b)+"\n")
    textList.append("fringe_size: " + str(len(a))+"\n")
    textList.append("max_fringe_size: " + str(e)+"\n")
    textList.append("search_depth: " + str(c.path_cost)+"\n")
    textList.append("max_search_depth: " + str(d)+"\n")
    textList.append("running_time: " + '{0:.8f}'.format(run_time)+"\n")
    textList.append("max_ram_usage: " + '{0:.8f}'.format(run_ram)+"\n")
    
    output = open('output.txt','w')
    output.writelines(textList)
    output.close()
    
if __name__ == "__main__":
    main()