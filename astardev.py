import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# flake8: noqa

nodetxt = [
"# Each line is of the form 'node,cost_to_go,heuristic_cost",
"1,0,0,0",
"2,12,4,0",
"3,10,2,0",
"4,10,-2,0",
"5,12,-4,0",
"6,20,0,0",
]

edgetxt = [
"# Each line is of the form 'node1,node2,cost'",
"1,3,18",
"1,4,12",
"1,5,30",
"3,2,27",
"3,6,15",
"4,6,20",
"4,5,8",
"2,6,10",
"5,6,10",
]
fnamenodes = ""
fnameedges = ""
firstnode = "1"
targetnode = "6"

astarscene = "PRM2"
astarscene = "Scene5"
astarscene = "PRM"
astarscene = ""
astarscene = "Scene5"
dname = ""


if astarscene=="Scene5":
    dname  = "Scene5"
    fnamenodes = "nodes.csv"
    fnameedges = "edges.csv"
    firstnode = "1"
    targetnode = "12"
elif astarscene=="PRM":
    dname = "PRM"
    fnamenodes = "nodes.csv"
    fnameedges = "edges.csv"
    firstnode = "1.000000"
    targetnode = "36.000000"
elif astarscene=="PRM2":
    dname = "PRM"
    fnamenodes = "nodes2.csv"
    fnameedges = "PRM/edges2.csv"
    firstnode = "1.000000"
    targetnode = "36.000000"

if fnamenodes!="":
    with open(f"{dname}/{fnamenodes}") as file:
        sizehint = 0
        nodetxt = file.readlines( sizehint )

    print(f"There are {len(nodetxt)} nodes")
    # print(nodetxt)

if fnameedges!="":
    with open(f"{dname}/{fnameedges}") as file:
        sizehint = 0
        edgetxt = file.readlines( sizehint )

    print(f"There are {len(edgetxt)} edges")


nodedict:dict[str,dict[str,float]]= {} 
nodestat:dict[str,str] = {}
parentnode:dict[str,str] = {}
nbr : dict[str,list[str]] = {}
edgecost : dict[str,float] = {}

def Initialize():
    global nodetxt, edgetxt, nodedict, edgecost, nbr
    for l in nodetxt:
        if l[0]=="#": continue
        if len(l)<=1: continue
        n,xv,yv,cst = l.split(",")
        nodedict[n] = {"x":float(xv),"y":float(yv),"cost":float(cst),"tent_tot_cost":float(cst)}
        nbr[n] = []
        nodestat[n] = "unvisited"
    # print("node",node)
    # print("nbr",nbr)

    mincost = 1e6
    maxcost = -1e6
    sumcost = 0
    for l in edgetxt:
        if l[0]=="#": continue
        if len(l)<=1: continue
        n1,n2,cost = l.split(",")
        nbr[n1].append(n2)
        nbr[n2].append(n1)
        if float(cost) < mincost: mincost = float(cost)
        if float(cost) > maxcost: maxcost = float(cost)
        sumcost += float(cost)
        edgecost[f"{n1}:{n2}"] = float(cost)
        edgecost[f"{n2}:{n1}"] = float(cost)
    print(f"mincost:{mincost:.3f} maxcost:{maxcost:.3f} avgcost:{sumcost/len(edgecost):.3f}")
    # print("edgecost",edgecost)

def Distance(n1:str,n2:str):
    global nodedict
    x1 = nodedict[n1]["x"]
    y1 = nodedict[n1]["y"]
    x2 = nodedict[n2]["x"]
    y2 = nodedict[n2]["y"]
    return numpy.sqrt((x1-x2)**2 + (y1-y2)**2)  

def CheckDistances():
    for e in edgecost.keys():
        n1,n2 = e.split(":")
        dist = Distance(n1,n2)
        if dist > edgecost[e]:
            print(f"CheckDist error dist>edgcost: {n1} {n2} {dist:.1f} {edgecost[e]}")
    

Initialize()
# CheckDistances()

def GetNodeColor(n:str):
    # Named colors: https://matplotlib.org/stable/gallery/color/named_colors.html
    if nodestat[n]=="open": return "lightseagreen"
    if nodestat[n]=="closed": return "darkgreen"
    if nodestat[n]=="unvisited": return "blue"
    return "black"

def PlotNodesWithNames(iplt:int):
    
    # Plot the nodes using matplotlib

    global nodedict,scale,astarscene,nrows,ncols,fig

    ax = fig.add_subplot(nrows,ncols,iplt, aspect='equal') # type: ignore

    xmin = 1e6
    xmax = -1e6
    ymin = 1e6
    ymax = -1e6
    for n in nodedict.keys():
        if nodedict[n]["x"] < xmin: xmin = nodedict[n]["x"]
        if nodedict[n]["x"] > xmax: xmax = nodedict[n]["x"]
        if nodedict[n]["y"] < ymin: ymin = nodedict[n]["y"]
        if nodedict[n]["y"] > ymax: ymax = nodedict[n]["y"]

    borderx = 0.1*(xmax-xmin)
    bordery = 0.1*(ymax-ymin)
    ax.set_xlim(xmin-borderx,xmax+borderx)
    ax.set_ylim(ymin-bordery,ymax+bordery)
    scale =(xmax-xmin) / 2
    ax.grid(True, which='both')
    for n in nodedict.keys():
        x = nodedict[n]["x"]
        y = nodedict[n]["y"]
        clr = GetNodeColor(n)
        ax.add_patch(patches.Circle((x,y), scale*0.07, color=clr))
        nodename = n.replace(".000000","") # remove trailing zeros if they are in the label
        ax.text(x,y,nodename,fontsize=10,horizontalalignment='center',verticalalignment='center',color='white')

    # Now add the links between the nodes with cost 
    for e in edgecost.keys():
        n1,n2 = e.split(":")
        x1 = nodedict[n1]["x"]
        y1 = nodedict[n1]["y"]
        x2 = nodedict[n2]["x"]
        y2 = nodedict[n2]["y"]
        ax.plot([x1,x2],[y1,y2],color='green',linewidth=1)
        # now calculate the midpoint and add the cost
        xmid = (x1+x2)/2
        ymid = (y1+y2)/2
        ax.text(xmid,ymid,f"{edgecost[e]:.3f}",fontsize=10,horizontalalignment='center',verticalalignment='center')

    plt.draw()

def HightlightNodesInPath(path: list[str],cost,actionline:str):

    # Highlight the path we found
    global nodedict
    import matplotlib.patches as patches    
    ax = plt.gca()
    tit = "-".join(path) + f" cost:{cost:.3f}\n{actionline}"
    ax.set_title(tit)
    # ax.title = tit
    for n in path:
        x = nodedict[n]["x"]
        y = nodedict[n]["y"]
        ax.add_patch(patches.Circle((x,y), scale*0.07, edgecolor='red', lw=4, fill=False))

fig = None
iplot:int = 1
nrows:int = 3
ncols:int = 6

def setupPlot(tit:str):
    global nodedict, iplot, nrows, ncols, fig
    iplot = 1
    fig = plt.figure()
    fig.suptitle(tit)    

def doSubPlot(n:str,actionline:str):
    global nodedict, iplot, nrows, ncols
    PlotNodesWithNames(iplot)
    curp = getParentList(n)
    curp.reverse()
    cost = astarcost(curp)
    iplot += 1
    HightlightNodesInPath(curp,cost,actionline)


    

def addNodeToOpenList(n:str,openlist: list[str]):
    global nodedict
    ntentcost = nodedict[n]["tent_tot_cost"]
    for i in range(len(openlist)):
        if ntentcost < nodedict[openlist[i]]["tent_tot_cost"]:
            openlist.insert(i,n)
            nodestat[n] = "open"
            # print(f"openlist:{openlist} {ntentcost}")
            return
    openlist.append(n)
    nodestat[n] = "open"
    # print(f"openlist:{openlist} {ntentcost}")

def addNodeToClosedList(n:str,closedlist: list[str]):
    closedlist.append(n)
    nodestat[n] = "closed"

def getParentList(n:str)->list[str]:
    global nodedict,parentnode
    rv = []
    while n in nodedict.keys():
        rv.append(n)
        if n not in parentnode.keys(): break
        n = parentnode[n]
    return rv

def getSeeminglyClosestNodeToTarget(openlist:list[str])->str:
    global nodedict
    if len(openlist)==1: return openlist[0]
    mincost = 1e6
    minnode :str = "None"
    for n in openlist:
        if "tent_tot_cost" not in nodedict[n].keys(): continue
        if nodedict[n]["tent_tot_cost"] < mincost:
            mincost = nodedict[n]["tent_tot_cost"]
            minnode = n
    return minnode

def assignParent(n:str,parent:str,goal:str):
    global nodedict, parentnode
    parentnode[n] = parent
    nodedict[n]["cost"] = nodedict[parent]["cost"] + edgecost[f"{parent}:{n}"]
    nodedict[n]["tent_tot_cost"] = nodedict[n]["cost"] + Distance(n,goal)


      
def astar(start:str,goal:str)->list[str]:
    global nodedict, nbr, edgecost
    openlist = [start]
    closedlist = []
    setupPlot(f"A* for {astarscene}")
    while len(openlist)>0:
        n :str = getSeeminglyClosestNodeToTarget(openlist)
        openlist.remove(n)
        if n==goal:
            rv = getParentList(n)
            rv.reverse()
            doSubPlot(n,f"Solution")
            return rv
        for n2 in nbr[n]: 
            if n2 in closedlist: continue
            if n2 not in openlist:
                assignParent(n2,n,goal)
                addNodeToOpenList(n2,openlist)
                doSubPlot(n2,f"added {n2} to openlist")
            else:
                # if the nodes is in the openlist then we might need to reset the costs if we found a better path
                if nodedict[n2]["cost"] > nodedict[n]["cost"] + edgecost[f"{n}:{n2}"]:
                    assignParent(n2,n,goal)
        addNodeToClosedList(n,closedlist)
        doSubPlot(n,f"added {n} to closed")

    return []

def astarcost(path)->float:
    global nodedict, edgecost
    cost = 0
    for i in range(len(path)-1):
        n1 = path[i]
        n2 = path[i+1]
        print(edgecost[f"{n1}:{n2}"])
        cost += edgecost[f"{n1}:{n2}"]
    return cost


rv = astar(firstnode,targetnode)
print("bestpath:",rv)
print(f"{astarcost(rv):.5f}")
plt.show()
altpath = ['1', '2', '5', '7', '10', '12']
print("altpath:",altpath)
print(f"{astarcost(altpath):.5f}")
altpath1 = ['1', '3', '4', '8', '12']
print("altpath1:",altpath1)
print(f"{astarcost(altpath1):.5f}")
altpath3 = ['1', '2', '5', '7', '10', '12']
print("altpath3:",altpath3)
print(f"{astarcost(altpath3):.5f}")

# Write out the solution node path to "path.csv"
nodestring = ",".join(rv)
with open('path.csv', 'w') as file:
    file.write(nodestring)