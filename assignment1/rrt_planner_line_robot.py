import time
import random
import drawSample
import math
from math import sqrt
import sys
import imageToRects
import utils

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
args = utils.get_args()
visualize = utils.get_args()
drawInterval = 100 # 10 is good for normal real-time drawing

prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = args.step_size # what our "local planner" can handle.
map_size,obstacles = imageToRects.imageToRects(args.world)
#Note the obstacles are the two corner points of a rectangle
#Each obstacle is (x1,y1), (x2,y2), making for 4 points
XMAX = map_size[0]
YMAX = map_size[1]

G = [  [ 0 ]  , [] ]   # nodes, edges
vertices = [ [args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

# goal/target
tx = args.target_pos_x
ty = args.target_pos_y

# start
sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0
nodes=0
edges=1

def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    # Calculates 'a' and 'b' values of a line equation given two points
    # y = a*x + b
    x = p1[0]
    y = p1[1]

    x1 = p2[0]
    y1 = p2[1]

    if (x - x1) == 0:
        a = 0
    else:
        a = (y - y1)/(x - x1)
    b = y - a*x

    return [a,b]


def pointPointDistance(p1,p2):
    # return distance p1 to p2
    return sqrt( (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 )

def takeSecond(elem):
    return elem[0]

def closestPointToPoint(G,p2):
    distance_to_point = float("inf")
    closest_node = None

    # Breath First Search in all nodes to find the closest node to point 2
    queue = []
    visited = []

    G[edges].sort(key=takeSecond) # Sorts edge list by first node in the edge pair

    #for edge in G[edges]:
    #    print("edge btw: " + str(edge[0]) + " and " +  str(edge[1]))

    queue.append(G[edges][0][0])  # Appending root of graph
    visited.append(G[edges][0][0]) # Adding to visited
    
    while queue:
        #print("queue: " + str(queue))
        node = queue.pop(0)
        #print("visited: " + str(visited))

        for edge in G[edges]:
        # Update distance and closest node
            if node == edge[0] and edge[1] not in visited:
                #print("got here!")
                #Calculate distance from that node and update closest node
                distance = pointPointDistance(vertices[edge[1]], p2)
                #print("distance; " + str(distance))
                if distance < distance_to_point:
                    distance_to_point = distance
                    closest_node = edge[1]

                visited.append(edge[1])
                queue.append(edge[1])

    #return vertex index
    return closest_node

def lineHitsRect(p1,p2,r, dilation):
    '''
    Check if line hits and points hit rectangle (obstacle)
    p1 = closest point
    p2 = new point
    '''

    line = lineFromPoints(p1, p2)
    a = line[0]
    b = line[1]

    #Getting rectangle
    x = r[0]
    y = r[1]
    x1 = r[2]
    y1 = r[3]
    p1_rect = [x,y]     #left bottom corner
    p2_rect = [x1,y1]   #right top corner
    p3_rect = [x1,y]    #right bottom corner
    p4_rect = [x,y1]    #left top corner

    rect_diagonal = lineFromPoints(p1_rect, p2_rect)
    rect_diagonal1 = lineFromPoints(p3_rect, p4_rect)

    interception_x = (rect_diagonal[1]- b)/ (a - rect_diagonal[0])
    interception_y = a * interception_x + b

    inter_point = [interception_x,interception_y]

    interception_x1 = (rect_diagonal1[1]- b)/ (a - rect_diagonal1[0])
    interception_y1 = a * interception_x1 + b
    
    inter_point1 = [interception_x1,interception_y1]

    if inRect(inter_point, r, dilation) == 1: # if interception point between line and first diagonal is inside the rectangle
        if inRect(p1, r, dilation) == 0 and inRect(p2, r, dilation) == 0:# if both points are outside the rectangle
            if p1[0] < interception_x and p2[0] > interception_x:           # if p1x < inter_x < p2x
                return True
            elif p1[1] < interception_y and p2[1] > interception_y:         # if p1y < inter_y < p2y
                return True
            elif p1[0] > interception_x and p2[0] < interception_x:         # if p2x < inter_x < p1x
                return True
            elif p1[1] > interception_y and p2[1] < interception_y:         # if p2y < inter_y < p1y
                return True
            else: return False
        else: return True
    
    if inRect(inter_point1, r, dilation) == 1: # if interception point between line and second diagonal is inside the rectangle
        if inRect(p1, r, dilation) == 0 and inRect(p2, r, dilation) == 0:# if both points are outside the rectangle
            if p1[0] < interception_x1 and p2[0] > interception_x1:         # if p1x < inter_x < p2x
                return True
            elif p1[1] < interception_y1 and p2[1] > interception_y1:       # if p1y < inter_y < p2y
                return True   
            elif p1[0] > interception_x1 and p2[0] < interception_x1:       # if p2x < inter_x < p1x
                return True
            elif p1[1] > interception_y1 and p2[1] < interception_y1:       # if p2y < inter_y < p1y
                return True         
            else: return False
        else: return True

    return False

def inRect(p,rect,dilation):
   """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
   if p[0]<rect[0]-dilation: return 0   # x < r_x1
   if p[1]<rect[1]-dilation: return 0   # y < r_y1
   if p[0]>rect[2]+dilation: return 0   # x > r_x2
   if p[1]>rect[3]+dilation: return 0   # y > r_y2
   return 1


def newPoint(p1,p2,stepsize, robotsize):

    stepsize = stepsize + robotsize
    # Calculate line from points
    #print("p1 x: " + str(p1[0]) + " y: " + str(p1[1]))
    line = lineFromPoints(p1, p2)
    #print("line: "+str(line))
    # Get angle
    angle = math.atan(line[0])  # angle in radians
    # Get new point coordinates

    new_x = p1[0] + (stepsize * math.cos(angle))
    new_y = p1[1] + (stepsize * math.sin(angle))
    
    new_point = [new_x, new_y]

    new_x1 = p1[0] + (-stepsize * math.cos(angle))
    new_y1 = p1[1] + (-stepsize * math.sin(angle))

    new_point1 = [new_x1, new_y1]

    if pointPointDistance(p2,new_point) < pointPointDistance(p2,new_point1):
        return new_point
    else: return new_point1

def robotTurn(point, angle, robot_size):

    new_x = point[0] + (robot_size * math.cos(angle))
    new_y = point[1] + (robot_size * math.sin(angle))
    
    new_point = [new_x, new_y]

    new_x1 = point[0] + (-robot_size * math.cos(angle))
    new_y1 = point[1] + (-robot_size * math.sin(angle))

    new_point1 = [new_x1, new_y1]

    if pointPointDistance(point,new_point) < pointPointDistance(point,new_point1):
        return new_point
    else: return new_point1



def rrt_search(G, tx, ty, canvas, robot_size):
    #Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    i = 0
    while 1: # Main loop
        # This generates a point in form of [x,y] from either the normal dist or the Gaussian dist
        
        if i % 15 == 0:    #every x steps random node is target node
            p = [tx, ty]
            #print("pulling towards goal: " + str(p))
        else:
            p = genPoint()
        i = i + 1
        
        

        #p = genPoint()
        #print("\np: " + str(p))
        # This function must be defined by you to find the closest point in the existing graph to the guiding point
        cp = closestPointToPoint(G,p) # This function must return index of node
        
        #print("Closest node = "+ str(cp) + " coordinates: " + str(vertices[cp]))



        new_p = newPoint(vertices[cp],p,SMALLSTEP, robot_size) # -> gets [x, y]
        
        #robot_point = robotTurn(new_p, angle, robot_size)
        #print(" new_P = "+ str(new_p))

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        hit = 0  # 0 -> no obstacle hit
        for o in obstacles:
            # The following function defined by you must handle the occlusion cases
            if  inRect(new_p,o,0) or lineHitsRect(vertices[cp],new_p,o, 1):
                #print ("New Point hits obstacle! Skipping to another point.")
                #print("rectangele: " + str(o)
                hit = 1
                break
            
            #if inRect(robot_point,o,0) or lineHitsRect(new_p,robot_point,o, 1): #Can robot move to new point knowing the robot length?
            #    print("robot movement not allowed! Skip point")
            #    hit = 1
            #    break
                #... reject Skip this point
        

        if hit == 1: # obstacle hit
            continue

        #print("Adding new point to Graph!")
        new_p = newPoint(vertices[cp],p,SMALLSTEP, 0)

        k = pointToVertex( new_p )   # k is the new vertex ID, this add new point to vertices list
        G[nodes].append(k)
        G[edges].append( (cp,k) )
        #print(str(G))
        
        if visualize:
            canvas.polyline(  [vertices[cp], vertices[k] ]  )

        if pointPointDistance(new_p, [tx,ty] ) < SMALLSTEP:
            print ("Target achieved.", nsteps, "nodes in entire tree")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([new_p, vertices[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print ("Path length", totaldist, "using", nsteps, "nodes.")

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print ("More [c,q,g,Y]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print ("[" + d + "]")
                    if d == "c": canvas.delete()
                    if d == "q": return 0
                    if d == "g": prompt_before_next = 0
                break
        #i = i + 1
        nsteps = nsteps + 1  # count steps

def main(robot_size):
    #seed
    #robot_size = 50
    # goal/target



    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    i = 1
    while i:
        # graph G
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )
        drawGraph(G, canvas)

        #print("graph:  " + str(G[edges]))
        i = rrt_search(G, tx, ty, canvas, robot_size)
        if i == 0:
            return
        #i = i + 1

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    
    
    
    main(50)
