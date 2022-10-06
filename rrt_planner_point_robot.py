'''
Hello and welcome to the first assignment :)
Please try to enjoy implementing algorithms you learned from the course; in this regard, we also have tried
to provide a good starting point for you to develop your own code. In this script, you may find different functions
that should be defined by you to perform specific tasks. A good suggestion would be to begin with the primary
function for the RRT algo named as "rrt_search". Following this function line by line you may realize how to define
each function to make it work!
Note, you may use the following commands to run this script with the required arguments:
python3 rrt_planner_point_robot.py --arg1_name your_input1 --arg2_name your_input2 e.g.
python3 rrt_planner_point_robot.py --world="shot.png"
To see the list of arguments, take a look at utils.py
Also note:
G is a list containing two groups: the first group includes the nodes IDs (0,1,2,...), while the second holds all pairs of nodes creating an edge
Vertices is a list of nodes locations in the map; it corresponds to the nodes' IDs mentioned earlier
GOOD LUCK!
'''


from math import sqrt, cos, sin, atan 
import math
import random
import drawSample
import sys
import imageToRects
import utils


def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        # e.g. vertices: [[10, 270], [10, 280]]
        canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


# Use this function to generate points randomly for the RRT algo
def genPoint():
    # if args.rrt_sampling_policy == "uniform":
    #     # Uniform distribution
    #     x = random.random()*XMAX
    #     y = random.random()*YMAX
    # elif args.rrt_sampling_policy == "gaussian":
    #     # Gaussian with mean at the goal
    #     x = random.gauss(tx, sigmax_for_randgen)
    #     y = random.gauss(ty, sigmay_for_randgen)
    # else:
    #     print ("Not yet implemented")
    #     quit(1)

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
    vertices.append(p)
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
    #TODO
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

#nodes = 0
#edges = 1
#G = [ [0,1,2,3] , [(0,2),(1,3),(0,1) ] ]
#vertices = [ [10,0], [20, 0], [0, 0], [40, 10] ]
#point = [20, 5]
#print(closestPointToPoint(G, point))


#for node in G[nodes]:
#    print("node:" + str(node))


def inRect(p,rect,dilation):
   """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
   if p[0]<rect[0]-dilation: return 0   # x < r_x1
   if p[1]<rect[1]-dilation: return 0   # y < r_y1
   if p[0]>rect[2]+dilation: return 0   # x > r_x2
   if p[1]>rect[3]+dilation: return 0   # y > r_y2
   return 1

def lineHitsRect(p1,p2,r, dilation):
    #TODO
    #dilation = 0

    line = lineFromPoints(p1, p2)
    a = line[0]
    b = line[1]

    #print("line: a: " + str(a) + " b: " + str(b))
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
    
    #print("diagonal: a: " + str(rect_diagonal[0]) + " b: " + str(rect_diagonal[1]))
    interception_x = (rect_diagonal[1]- b)/ (a - rect_diagonal[0])
    interception_y = a * interception_x + b
    #print("interception x: " + str(interception_x) + " y: " + str(interception_y))
    inter_point = [interception_x,interception_y]

    
    interception_x1 = (rect_diagonal1[1]- b)/ (a - rect_diagonal1[0])
    interception_y1 = a * interception_x1 + b
    
    inter_point1 = [interception_x1,interception_y1]


    if inRect(inter_point1, r, dilation) == 1 or inRect(inter_point1, r, dilation) == 1:
        return True
    
    return False

#p1 = [0, 0]
#p2 = [1.9, 1]
#r = [2, 0, 3, 1]
#print("Line hits rect? " + str(lineHitsRect(p1, p2, r)))


def newPoint(p1,p2,stepsize):

    # Calculate line from points
    #print("p1 x: " + str(p1[0]) + " y: " + str(p1[1]))
    line = lineFromPoints(p1, p2)
    print("line: "+str(line))
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

#p1 = [3,3]
#p2 = [0,0]

#print(str(newPoint(p1,p2,1)))
    # check if can add i.e not in rect
    #if lineHitsRect(p1, new_point) == False:
        
        #nodeID = pointToVertex(new_point)   # Add new point to vertices list
        #G[nodes].append(nodeID)             # Add to nodeID in G[nodes]
        #G[edges].append([])
        
        # add to Edge [closest node, new node]

       # return new_point
    #else:
    #    return -1



def rrt_search(G, tx, ty, canvas):
    # Please carefully read the comments to get clues on where to start
    #TODO
    #Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    i = 0
    while 1: # Main loop
        # This generates a point in form of [x,y] from either the normal dist or the Gaussian dist
        if i % 10 == 0:    #every 20 steps random node is target node
            p = [tx, ty]
            print("pulling towards goal: " + str(p))
        else:
            p = genPoint()
        i = i + 1
        print("\np: " + str(p))
        # This function must be defined by you to find the closest point in the existing graph to the guiding point
        cp = closestPointToPoint(G,p) # This function must return index of node
        
        print("Closest node = "+ str(cp) + " coordinates: " + str(vertices[cp]))
        new_p = newPoint(vertices[cp],p,SMALLSTEP) # -> gets [x, y]
        print(" new_P = "+ str(new_p))

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        hit = 0  # 0 -> no obstacle hit
        for o in obstacles:
            # The following function defined by you must handle the occlusion cases
            if  inRect(new_p,o,1) or lineHitsRect(vertices[cp],new_p,o, 1):
                print ("New Point hits obstacle! Skipping to another point.")
                print("rectangele: " + str(o))
                hit = 1
                break
                
                #... reject Skip this point
        

        if hit == 1: # obstacle hit
            continue

        print("did i got here?")
        k = pointToVertex( new_p )   # k is the new vertex ID, this add new point to vertices list
        G[nodes].append(k)
        G[edges].append( (cp,k) )
        print(str(G))
        
        if visualize:
            canvas.polyline(  [vertices[cp], vertices[k] ]  )

        if pointPointDistance(p, [tx,ty] ) < SMALLSTEP:
            print ("Target achieved.", nsteps, "nodes in entire tree")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
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
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                break
        i = i + 1


def main():
    #seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    
    i = 0
    while i < 10:
        # graph G
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )
        drawGraph(G, canvas)

        print("graph:  " + str(G[edges]))
        rrt_search(G, tx, ty, canvas)
        i = i + 1
    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    #'''
    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    #SMALLSTEP = args.step_size  # what our "local planner" can handle.
    SMALLSTEP = 30
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]

    print("Map size: "+ str(XMAX) + " , " + str(YMAX))
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

    # goal/target
    #tx = args.target_pos_x
    #ty = args.target_pos_y

    tx = 490
    ty = 370

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
    
    #'''
    
    
    
