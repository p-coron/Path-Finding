
# coding: utf-8

# In[ ]:

'''
Priscilla Coronado and Herschel Darko
CS365 Lab A
PCHD-labA.py
'''
import copy
from queue import *
import heapq
from collections import deque
import sys
#-----------------------------Searches functions-----------------------------------------------------------------------

def single_dfs(file):
    percept=world_gen(file)
    state=None
    seqact=None
    goal=None
    problem=None
    state=update_state(state,percept) 
    if not seqact:
        goal=formulate_goal(state)
        problem=formulate_problem(percept[0],state,goal)
        frontier=[]
        expanded_nodes=0
        explored_list=[]
        frontier.append(Node(state,None,0,None))
        while frontier:
            node=frontier.pop()
            if (node.state not in explored_list):
                explored_list.append(node.state)
            if problem.goal_test(node.state):
                final_node=node
                seqact=problem.solution(node)
                break
            for act in problem.actions(node.state):
                final_node=node
                childnode=node.child_node(problem,act)
                expanded_nodes+=1
                if ((childnode.state not in explored_list) and (childnode not in frontier)):
                        frontier.append(childnode)
        if seqact:
            print("The number of nodes expanded: ",expanded_nodes)
            print("\nThe path of the solution: ",final_node.path_totalcost())
            finalworld=solvedworldpartone(percept[0],seqact)
            print("\nThe solution of agent\n:")
            displayworld(finalworld)
            
            
            
            
          
            
def single_bfs(file):
    percept=world_gen(file)
    state=None
    seqact=None
    goal=None
    problem=None
    state=update_state(state,percept) 
    if not seqact:
        goal=formulate_goal(state)
        problem=formulate_problem(percept[0],state,goal)
        frontier=deque([])
        expanded_nodes=0
        explored_list=[]
        frontier.append(Node(state,None,0,None))
        while frontier:
            node=frontier.popleft()
            if (node.state not in explored_list):
                explored_list.append(node.state)
            if problem.goal_test(node.state):
                final_node=node
                seqact=problem.solution(node)
                break
            for act in problem.actions(node.state):
                final_node=node
                childnode=node.child_node(problem,act)
                expanded_nodes+=1
                if ((childnode.state not in explored_list) and (childnode not in frontier)):
                        frontier.append(childnode)
        if seqact:
            print("The number of nodes expanded: ",expanded_nodes)
            print("\nThe path of the solution: ",final_node.path_totalcost())
            finalworld=solvedworldpartone(percept[0],seqact)
            print("\nThe solution of agent\n:")
            displayworld(finalworld)
            
            
def single_gbfs(file):    
    percept=world_gen(file)
    state=None
    seqact=None
    goal=None
    problem=None
    state=update_state(state,percept)
    if not seqact:
        goal=formulate_goal(state)
        problem=formulate_problem(percept[0],state,goal)
        frontier=[]
        expanded_nodes=0
        ancestry=[]
        parent=Node(state,None,0,None)
        while(not problem.goal_test(parent.state)):
            ancestry.append(parent)
            pos_actions= problem.actions(parent.state)
            unique=False
            Action=PQueue()
            nextparent=None
            for act in pos_actions:
                childnode=parent.child_node(problem,act)
                if (problem.goal_test(childnode.state)):
                    sequact=problem.solution(childnode)
                    print("The number of nodes expanded: ",expanded_nodes)
                    print("\nThe path of the solution: ",childnode.path_totalcost())
                    print("\nThe solution of agent\n:")
                    finalworld=solvedworldpartone(percept[0],sequact)
                    displayworld(finalworld)
                    return 0
                prize=closestprize(parent,act)
                
                dist=ManDist(parent,act,prize)
                Action.push(dist,childnode)
            while not(unique):
                    nextparent=Action.pop()
                    if (nextparent):
                        newx=nextparent.state[0][0]
                        newy=nextparent.state[0][1]
                        numofprizes=nextparent.state[1]
                       
                        for ancestor in ancestry:
                        
                            if ((int(ancestor.state[0][0])==(int((newx))))):
                           
                                if ((int((ancestor.state[0][1])))==(int((newy)))):
                              
                                    if ((len(ancestor.state[1]))==(len(numofprizes))):
                                           
                                            unique=False
                                            break
                                    else:
                                        unique=True
                                else:
                                    unique=True
                            else:
                                unique=True
                    else:
                    
                        parent=parent.parent
                        Action=PQueue()
                        pos_actions= problem.actions(parent.state)
                        for act in pos_actions:
                            prize=closestprize(parent,act)
                            childnode=parent.child_node(problem,act)
                            dist=ManDist(parent,act,prize)
                            Action.push(dist,childnode)

            if (unique):
                if(problem.goal_test(nextparent.state)):
                    sequact=problem.solution(nextparent)
                    print("The number of nodes expanded: ",expanded_nodes)
                    print("\nThe path of the solution: ",nextparent.path_totalcost())
                    print("\nThe solution of agent\n:")
                    finalworld=solvedworldpartone(percept[0],sequact)
                    displayworld(finalworld)
                    break
                expanded_nodes+=1
                parent=nextparent
                Action=PQueue()

            

#-------------------------------------------------------------------------
    
def single_astar(file):    
    percept=world_gen(file)
    state=None
    seqact=None
    goal=None
    problem=None
    state=update_state(state,percept)
    if not seqact:
        goal=formulate_goal(state)
        problem=formulate_problem(percept[0],state,goal)
        frontier=[]
        expanded_nodes=0
        ancestry=[]
        parent=Node(state,None,0,None)
        while(not problem.goal_test(parent.state)):
            ancestry.append(parent)
            pos_actions= problem.actions(parent.state)
            unique=False
            Action=PQueue()
            nextparent=None
            for act in pos_actions:
                prize=closestprize(parent,act)
                childnode=parent.child_node(problem,act)
                if (problem.goal_test(childnode.state)):
                    sequact=problem.solution(childnode)
                    print("The number of nodes expanded: ",expanded_nodes)
                    print("\nThe path of the solution: ",childnode.path_totalcost())
                    print("\nThe solution of agent\n:")
                    finalworld=solvedworldpartone(percept[0],sequact)
                    displayworld(finalworld)
                    return 0
                dist=ManDist(parent,act,prize)+(childnode.path_totalcost()+1)
                Action.push(dist,childnode)
            while not(unique):
                    nextparent=Action.pop()
                    if (nextparent):
                        newx=nextparent.state[0][0]
                        newy=nextparent.state[0][1]
                        numofprizes=nextparent.state[1]
                       
                        for ancestor in ancestry:
                        
                            if ((int(ancestor.state[0][0])==(int((newx))))):
                           
                                if ((int((ancestor.state[0][1])))==(int((newy)))):
                              
                                    if ((len(ancestor.state[1]))==(len(numofprizes))):
                                           
                                            unique=False
                                            break
                                    else:
                                        unique=True
                                else:
                                    unique=True
                            else:
                                unique=True
                    else:
                    
                        parent=parent.parent
                        Action=PQueue()
                        pos_actions= problem.actions(parent.state)
                        for act in pos_actions:
                            prize=closestprize(parent,act)
                            childnode=parent.child_node(problem,act)
                            dist=ManDist(parent,act,prize)
                            Action.push(dist,childnode)

            if (unique):
                if(problem.goal_test(nextparent.state)):
                    sequact=problem.solution(nextparent)
                    print("The number of nodes expanded: ",expanded_nodes)
                    print("\nThe path of the solution: ",nextparent.path_totalcost())
                    print("\nThe solution of agent\n:")
                    finalworld=solvedworldpartone(percept[0],sequact)
                    displayworld(finalworld)
                    break
                expanded_nodes+=1
                parent=nextparent
                Action=PQueue()
            
def multi_astar(file):   
    percept=world_gen(file)
    state=None
    seqact=None
    goal=None
    problem=None
    state=update_state(state,percept)
    if not seqact:
        goal=formulate_goal(state)
        problem=formulate_problem(percept[0],state,goal)
        frontier=[]
        expanded_nodes=0
        ancestry=[]
        parent=Node(state,None,0,None)
        while(not problem.goal_test(parent.state)):
            ancestry.append(parent)
            pos_actions= problem.actions(parent.state)
            unique=False
            Action=PQueue()
            nextparent=None
            for act in pos_actions:
                prize=closestprize(parent,act)
                childnode=parent.child_node(problem,act)
                if (problem.goal_test(childnode.state)):
                    print(childnode.state)
                    sequact=problem.solution(childnode)
                    print("The number of nodes expanded: ",expanded_nodes)
                    print("\nThe path of the solution: ",childnode.path_totalcost())
                    print("\nThe solution of agent\n:")
                    finalworld=solvedworldpartone(percept[0],sequact)
                    displayworld(finalworld)
                    return 0
                dist=Heuristic(parent,act,prize)+(childnode.path_totalcost()+1)
                Action.push(dist,childnode)
            while not(unique):
                    nextparent=Action.pop()
                    if (nextparent):
                        newx=nextparent.state[0][0]
                        newy=nextparent.state[0][1]
                        numofprizes=nextparent.state[1]
                       
                        for ancestor in ancestry:
                        
                            if ((int(ancestor.state[0][0])==(int((newx))))):
                           
                                if ((int((ancestor.state[0][1])))==(int((newy)))):
                              
                                    if ((len(ancestor.state[1]))==(len(numofprizes))):
                                           
                                            unique=False
                                            break
                                    else:
                                        unique=True
                                else:
                                    unique=True
                            else:
                                unique=True
                    else:
                    
                        parent=parent.parent
                        Action=PQueue()
                        pos_actions= problem.actions(parent.state)
                        for act in pos_actions:
                            prize=closestprize(parent,act)
                            childnode=parent.child_node(problem,act)
                            dist=ManDist(parent,act,prize)
                            Action.push(dist,childnode)

            if (unique):
                if(problem.goal_test(nextparent.state)):
                    sequact=problem.solution(nextparent)
                    print("The number of nodes expanded: ",expanded_nodes)
                    print("\nThe path of the solution: ",nextparent.path_totalcost())
                    print("\nThe solution of agent\n:")
                    finalworld=solvedworldpartone(percept[0],sequact)
                    displayworld(finalworld)
                    break
                expanded_nodes+=1
                parent=nextparent
                Action=PQueue()
    
#------------------------------------------------Helper functions------------------------------------------------------     

def Heuristic(node,action,prize):
    x=action[0]
    y=action[1]
    x=(prize[0]-x)
    y=(prize[0]-y)
    x=x**2
    y=x**2
    dis=(x+y)**(0.5)
    return dis
    
    
def solvedworldpartone(world,solution):
    for action in solution:
        worldrow=world[action[1]]
        worldrow[action[0]]="#"
    return(world)

def displayworld(world):
    for i in world:
        print("\n")
        for ik in i:
            print(ik,end="")



def update_state(state,percept):
    return (percept[1],percept[2])

def formulate_goal(state):
    return []    

def formulate_problem(world,state,goal):
    return Problem(world,state,goal)



def ManDist(node,action,prize):
    x=action[0]
    y=action[1]
    dis=(abs(x-prize[0])+abs(y-prize[0]))  
    return dis

def closestprize(node,act):
    prizes=node.state[1]
    shdist=sys.maxsize
    bprize=None
    for prize in prizes:
        x=prize[0]
        y=prize[1]
        dis=(abs((node.state[0][0]+act[0])-prize[0])+abs(abs((node.state[0][1]+act[1])-prize[1])))
        if (dis<shdist):
            bprize=prize
            shdist=dis
    #print("We chose this",bprize)
    return bprize



def world_gen (fname):
    with open(fname) as file:
        pallet_town = []
        poke_prizes=[]
        agentloc=None
        x=0
        y=-1
        for line in file:
            line=line.rstrip("\n")                      
            grass=[]
            y+=1
            x=0
            for i in line:
                if i==".":
                    poke_prizes.append((x,y))
                grass.append(i)
                
                if i=="P":
                    agentloc=(x,y)
                x+=1   
            pallet_town.append(grass)
    return (pallet_town,agentloc,poke_prizes )


#----------------------Classes--------------------------------------------------

class Problem():
    
    def __init__(self,world,state,goal=None,):
        self.state=state
        self.goal=goal
        self.world=world
       
        
    def actions(self,state):
        x=state[0][0]
        y=state[0][1]
        left=(x-1,y)
        right=(x+1,y)
        up=(x,y+1)
        down=(x,y-1)
        wall=[]
        actions=[left,right,up,down]
        for act in actions:    
            worldrow=self.world[act[1]]
            if ((worldrow[act[0]])=="%"):
                wall.append(act)
        for act in wall:
            actions.remove(act)
        return actions
    
    def transmodel(self,state,action):
        poke_prizes=state[1]
        movey=self.world[action[1]]
        movex=movey[action[0]]
        if movex==".":
            if action in poke_prizes:
                poke_prizes.remove(action)
                poke=[]
                for i in poke_prizes:
                    poke.append(i)
                return (action,poke)
        return (action,poke_prizes)
            
            
        
    def goal_test(self, state):
        return (state[1]==self.goal)
               
    def solution(self, Node):
        solution=[]
        solution.append(Node.state[0])
        while (Node.path_cost!=0):
            solution.append(Node.action)
            Node=Node.parent
        solution.reverse()
        return(solution)
            
        
class Node():
    
    def __init__(self,state, parent=None, path_cost=0, action=None):
    
        self.parent = parent
        self.path_cost = path_cost
        self.action = action
        self.state = self.copystate(state)
        
    def pos_actions(self, problem):
        return[self.child_node(problem, action) for action in problem.actions(self.state)]
    
    def child_node(self, problem, action):
        new_state = problem.transmodel(self.state, action)
        return Node(new_state,self,self.path_cost+1, action )
    
    def path_totalcost(self):
        return(self.path_cost)
    
    
    def copystate(self,state):
        x=state[0][0]
        y=state[0][1]
        prizelist=copy.deepcopy(state[1])
        return ((x,y),prizelist)
    


class PQueue:
    def __init__(self):
        self._queue = []
        self._index = 0

    def push(self, priority,item):
        heapq.heappush(self._queue, (-priority, self._index, item))
        self._index += 1

    def pop(self):
        self._index-=1
        if not(self.isEmpty()):
            return heapq.heappop(self._queue)[-1]
        else:
            return False
    
    def isEmpty(self):
        return( self._index ==-1)
    
    def Index(self):
        return self._index
    
if __name__ == "__main__":
    
    file = input("Enter file: ")
    
    single_astar(file)
    single_bfs(file)
    single_dfs(file)
    single_gbfs(file)
    multi_astar(file)  



    


# #####             
#             
# 

# In[ ]:



