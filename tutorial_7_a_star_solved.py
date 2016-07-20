# ------------------------------------------------------------------------------------------------
# Copyright (c) 2016 Microsoft Corporation
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all copies or
# substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ------------------------------------------------------------------------------------------------

# A* Search Solution for Tutorial sample #7: The Maze Decorator

# Note: I wrote this code quickly, and did a simple pass to make sure it's somewhat readable.
# Probably not as efficient or optimized as it could be (almost sure of it).
# I bet there are multiple places it could be improved, feel free to do whatever you'd like with it.

import MalmoPython
import os
import sys
import time
import json
import copy
from MIDCA import goals

sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)  # flush print output immediately

def GetMissionXML( seed, gp ):
    return '''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
            <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
            
              <About>
                <Summary>Hello world!</Summary>
              </About>
              
            <ServerSection>
              <ServerInitialConditions>
                <Time>
                    <StartTime>1000</StartTime>
                    <AllowPassageOfTime>false</AllowPassageOfTime>
                </Time>
                <Weather>clear</Weather>
              </ServerInitialConditions>
              <ServerHandlers>
                  <FlatWorldGenerator generatorString="3;7,44*49,73,35:1,159:4,95:13,35:13,159:11,95:10,159:14,159:6,35:6,95:6;12;"/>
                  <DrawingDecorator>
                    <DrawSphere x="-27" y="70" z="0" radius="30" type="air"/>
                  </DrawingDecorator>
                  <MazeDecorator>
                    <Seed>'''+str(seed)+'''</Seed>
                    <SizeAndPosition width="10" length="10" height="10" xOrigin="-32" yOrigin="69" zOrigin="-5"/>
                    <StartBlock type="emerald_block" fixedToEdge="true"/>
                    <EndBlock type="redstone_block" fixedToEdge="true"/>
                    <PathBlock type="diamond_block"/>
                    <FloorBlock type="air"/>
                    <GapBlock type="air"/>
                    <GapProbability>'''+str(gp)+'''</GapProbability>
                    <AllowDiagonalMovement>false</AllowDiagonalMovement>
                  </MazeDecorator>
                  <ServerQuitFromTimeUp timeLimitMs="30000"/>
                  <ServerQuitWhenAnyAgentFinishes/>
                </ServerHandlers>
              </ServerSection>
              
              <AgentSection mode="Survival">
                <Name>MalmoTutorialBot</Name>
                <AgentStart>
                    <Placement x="0.5" y="56.0" z="0.5"/>
                </AgentStart>
                <AgentHandlers>
                    <DiscreteMovementCommands/>
                    <AgentQuitFromTouchingBlockType>
                        <Block type="redstone_block"/>
                    </AgentQuitFromTouchingBlockType>
                    
                    <ObservationFromGrid>
                        <Grid name="front20x10">
                            <min x="-10" y="-1" z="0"/>
                            <max x="10" y="-1" z="10"/>
                        </Grid>
                    </ObservationFromGrid>
                </AgentHandlers>
              </AgentSection>
            </Mission>'''

class Node():
    '''
    a node that will be used in A* search
    '''
    agent_loc = None # x, y coordinate
    state = None # all_tiles in a grid
    parent_node = []
    actions_taken = [] # actions taken to reach this node
    depth = 0
    
    def __init__(self, agent_loc, state, parent_node, actions_taken):
        self.agent_loc = agent_loc
        self.state = state
        self.parent_node = parent_node
        if parent_node:
            self.depth = parent_node.depth+1
        else:
            self.depth = 0
        self.actions_taken = actions_taken

    def __str__(self):
        s = "aloc="+str(self.agent_loc)+"state_x_len="+str(len(self.state))+"state_y_len"+str(len(self.state[0]))
        return s
    
def get_child_nodes(node, curr_nodes, already_visited_nodes):
    if not node: # sanity check
        return []
    
    x = node.agent_loc[0]
    y = node.agent_loc[1]
    
    valid_child_nodes = []
    
    # add each action
    if x != len(grid[y]):
        # make a copy just to make sure we are not modifying some
        # node some other place
        e_actions = copy.deepcopy(node.actions_taken) 
        e_actions.append("movewest 1")
        east_node = Node((x+1,y),grid,node,e_actions)
        valid_child_nodes.append(east_node) #east node
    if x != 0:
        w_actions = copy.deepcopy(node.actions_taken)
        w_actions.append("moveeast 1")
        west_node = Node((x-1,y),grid,node,w_actions)
        valid_child_nodes.append(west_node) # west node
    if y != len(grid):
        s_actions = copy.deepcopy(node.actions_taken)
        s_actions.append("movesouth 1")
        south_node = Node((x,y+1),grid,node,s_actions)
        valid_child_nodes.append(south_node) #east node
    if y != 0:
        n_actions = copy.deepcopy(node.actions_taken)
        n_actions.append("movenorth 1")
        north_node = Node((x,y-1),grid,node,n_actions)
        valid_child_nodes.append(north_node) # west node
        
    # filter out anything that is air
    try:
        valid_child_nodes = filter(lambda n: n.state[n.agent_loc[1]][n.agent_loc[0]] != u'air', valid_child_nodes)
    except:
        pass
            
    # filter out anything that is already in curr_nodes (may not be necessary)
    curr_node_locs = map(lambda n: n.agent_loc,curr_nodes)
    valid_child_nodes = filter(lambda n: n.agent_loc not in curr_node_locs, valid_child_nodes)
    
    # filter out anything that we have already visited (necessary, prevents cycles)
    visited_node_locs = map(lambda n: n.agent_loc,already_visited_nodes)
    valid_child_nodes = filter(lambda n: n.agent_loc not in visited_node_locs, valid_child_nodes)
    return valid_child_nodes
        
def A_star_search(start_state):
    # find the location of the emerald block, record this as agent's position and start
    # also find location of redstone block, save as goal loc
    agent_loc_x = -1
    agent_loc_y = -1
    goal_x = -1
    goal_y = -1
    found_emerald_block = False
    found_redstone_block = False
    for y in range(len(start_state)):
        for x in range(len(start_state[y])):
            if start_state[y][x] == u'emerald_block':
                agent_loc_x = x
                agent_loc_y = y
                found_emerald_block = True
            elif start_state[y][x] == u'redstone_block':
                goal_x = x
                goal_y = y
                found_redstone_block = True
                
    # just in case our state isn't valid
    if not (found_emerald_block and found_redstone_block):
        return []
    
    print "agent_loc = " +str(agent_loc_x) + ","+str(agent_loc_y)
    print "goal_loc = "+str(goal_x) + ","+str(goal_y)
    
    # safety check, make sure start and goal are not the same
    if str(agent_loc_x) + ","+str(agent_loc_y) == str(goal_x) + ","+str(goal_y):    
        return []
    
    # root node
    root_node = Node((agent_loc_x, agent_loc_y),grid,None,[])
    
    def goal_reached(node):
        reached = False
        try:
            reached = grid[node.agent_loc[1]][node.agent_loc[0]] == u'redstone_block'
        except:
            print "somehow it broked with y="+str(node.agent_loc[1])+", x="+str(node.agent_loc[1])  
        return reached
    
    def manhattan_dist_to_goal(node):
        #print "g(n)="+str((abs(goal_x - node.agent_loc[0])+abs(goal_y-node.agent_loc[1])))+",h(n)="+str(node.depth)
        return (abs(goal_x - node.agent_loc[0])+abs(goal_y-node.agent_loc[1])) + node.depth
    
    def depth_and_manhatten(node):
        return manhattan_dist_to_goal(node) 

    # initialize the queue with the first node
    curr_nodes = [root_node]
    already_visited_nodes = []
    while len(curr_nodes) > 0 and not goal_reached(curr_nodes[0]):
        # get first node from our queue
        curr_node = curr_nodes[0]
        curr_nodes = curr_nodes[1:] # take the first node off
        
        # save this node so we don't visit again
        already_visited_nodes.append(curr_node)
        
        # get child nodes
        child_nodes = get_child_nodes(copy.deepcopy(curr_node),curr_nodes, already_visited_nodes)
                
        # add child nodes to queue
        curr_nodes += child_nodes
        
        # sort queue based on manhatten + depth
        curr_nodes = sorted(curr_nodes,key=depth_and_manhatten)
        
        #print "already_visited_nodes = "+str(map(lambda n: n.agent_loc, already_visited_nodes))
        print "Q = " +str(map(lambda n: "f(n)="+str(depth_and_manhatten(n))+",xy="+str(n.agent_loc), curr_nodes))
        #print "queue (depths) = "+str(map(lambda n: n.depth, curr_nodes))
        #print "queue size = " +str(len(curr_nodes))
        #print "queue distances to goal: "+str(map(depth_and_manhatten,curr_nodes))
        # sort nodes based on depth (bfs for now)
        
        #time.sleep(0.1)
       
    # make sure goal was reached
    if not goal_reached(curr_nodes[0]):
        print "ERROR: search terminated without finding a path"
    else:
        print "computed path:"
        for action in curr_nodes[0].actions_taken:
            print "  " + str(action)
        return curr_nodes[0].actions_taken

def pretty_print_grid_obs(grid):
    '''
    Displays the state used in A* nodes
    '''
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j] == u'air':
                print 'a',
            elif grid[i][j] == u'diamond_block':
                print 'd',
            elif grid[i][j] == u'emerald_block':
                print 'E',
            elif grid[i][j] == u'redstone_block':
                print 'R',
            else:
                print '?',
        print ""

# Create default Malmo objects:
time.sleep(5) # helps recording the video
agent_host = MalmoPython.AgentHost()
try:
    agent_host.parse( sys.argv )
except RuntimeError as e:
    print 'ERROR:',e
    print agent_host.getUsage()
    exit(1)
if agent_host.receivedArgument("help"):
    print agent_host.getUsage()
    exit(0)

if agent_host.receivedArgument("test"):
    num_repeats = 1
else:
    num_repeats = 10

num_repeats = 10
for i in range(num_repeats):
    my_mission = MalmoPython.MissionSpec(GetMissionXML("random", float(i/10.0)), True)
    my_mission_record = MalmoPython.MissionRecordSpec()

    #agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)

    # Attempt to start a mission:
    max_retries = 3
    for retry in range(max_retries):
        try:
            agent_host.startMission( my_mission, my_mission_record )
            break
        except RuntimeError as e:
            if retry == max_retries - 1:
                print "Error starting mission:",e
                exit(1)
            else:
                time.sleep(2)

    # Loop until mission starts:
    print "Waiting for the mission to start ",
    world_state = agent_host.getWorldState()
    while not world_state.is_mission_running:
        sys.stdout.write(".")
        time.sleep(0.1)
        world_state = agent_host.getWorldState()
        for error in world_state.errors:
            print "Error:",error.text

    print
    print "Mission running ",

    # Loop until mission ends:
    while world_state.is_mission_running:
        sys.stdout.write(".")
        time.sleep(0.1)
        world_state = agent_host.getWorldState()
        
        #print "observations are " + str(world_state.observations)
        
        if world_state.number_of_observations_since_last_state > 0:
            #print "Got " + str(world_state.number_of_observations_since_last_state) + " observations since last state."
            msg = world_state.observations[-1].text
            ob = json.loads(msg)
            #print "ob is "+str(ob)
            
            '''
            I used a special observation to get 200 blocks in front of the agent,
            which I know will contain the maze (and extra air tiles). I think perform
            A* over this grid of blocks.
            '''
            # get the data in front of the agent ONCE
            all_tiles = ob.get(u'front20x10',0)
            grid = []
            for i in range(10):
                #print "looking at all_tiles["+str((i*20))+":"+str(((i+1)*20))+"][::1]"
                # for some reason it's reverse (might have to do with agent's yaw)
                reverse_row = (all_tiles[i*21:((i+1)*21)])[::-1] 
                #print "len(row) = " + str(len(reverse_row))
                grid.append(reverse_row)
            
            # lets see what it looks like
            pretty_print_grid_obs(grid)
            
            # run A* and compute plan
            plan = A_star_search(grid)
            if plan:
                for a in plan:
                    # do it
                    agent_host.sendCommand(a)
                    time.sleep(0.5) 
            
                print "We should be at our goal!"
            
            
        for error in world_state.errors:
            print "Error:",error.text

    print
    print "Mission ended"
    # Mission has ended.
    time.sleep(0.5)


    