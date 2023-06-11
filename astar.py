import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
# f l a k e 8 : noqa


class AStar:

    nodedict: dict[str, dict[str, float]] = {}
    nodestat: dict[str, str] = {}
    parentnode: dict[str, str] = {}
    nbr: dict[str, list[str]] = {}
    edgecost: dict[str, float] = {}

    def __init__(self, nodetxt: list[str], edgetxt: list[str]):
        print("AStar.__init__")
        print("nodetxt:", nodetxt)
        print("edgetxt:", edgetxt)
        if (len(nodetxt) == 1):
            nodetxt = self.FileToList(nodetxt[0])
        if (len(edgetxt) == 1):
            edgetxt = self.FileToList(edgetxt[0])

        print("There are", len(nodetxt), "nodes")
        print("There are", len(edgetxt), "edges")
        for line in nodetxt:
            if line[0] == "#":
                continue
            if len(line) <= 1:
                continue
            n, xv, yv, cst = line.split(",")
            self.nodedict[n] = {"x": float(xv),
                                "y": float(yv),
                                "cost": float(cst),
                                "tent_tot_cost": float(cst)}
            self.nbr[n] = []
            self.nodestat[n] = "unvisited"
        # print("nodedict", self.nodedict)
        # print("nbr", self.nbr)

        mincost: float = 1e6
        maxcost: float = -1e6
        sumcost = 0
        for line in edgetxt:
            if line[0] == "#":
                continue
            if len(line) <= 1:
                continue
            n1, n2, cost = line.split(",")
            if n1 not in self.nbr:
                print(f"Error in edgelist: ""{n1}"" is not a node")
                continue
            if n2 not in self.nbr:
                print(f"Error in edgelist: ""{n2}"" is not a node")
                continue
            self.nbr[n1].append(n2)
            self.nbr[n2].append(n1)
            fcost = float(cost)
            mincost = min(mincost, fcost)
            maxcost = max(maxcost, fcost)
            sumcost += fcost
            self.edgecost[f"{n1}:{n2}"] = fcost
            self.edgecost[f"{n2}:{n1}"] = fcost
        nedges = max(1, len(self.edgecost))
        print(f"mincost:{mincost:.3f} maxcost:{maxcost:.3f} avgcost:{sumcost/nedges:.3f}")
        # print("edgecost",edgecost)

    def FileToList(self, fname: str) -> list[str]:
        if (os.path.isfile(fname)):
            with open(fname) as f:
                flist = f.readlines()
                return flist
        else:
            return []

    def Distance(self, n1: str, n2: str):
        n1 = self.nodedict[n1]
        x1 = n1["x"]
        y1 = n1["y"]
        n2 = self.nodedict[n2]
        x2 = n2["x"]
        y2 = n2["y"]
        return numpy.sqrt((x1-x2)**2 + (y1-y2)**2)

    def CheckDistances(self):
        for e in self.edgecost.keys():
            n1, n2 = e.split(":")
            dist = self.Distance(n1, n2)
            if dist > self.edgecost[e]:
                msg = f"CheckDist error dist>edgcost: {n1} {n2} {dist:.1f} {self.edgecost[e]}"
                print(msg)

    def GetNodeColor(self, n: str):
        # Named colors: https://matplotlib.org/stable/gallery/color/named_colors.html
        if self.nodestat[n] == "open":
            return "lightseagreen"
        if self.nodestat[n] == "closed":
            return "darkgreen"
        if self.nodestat[n] == "unvisited":
            return "blue"
        return "black"

    def PlotNodesWithNames(self, iplt: int, isSubplot=True):

        # Plot the nodes using matplotlib

        if isSubplot:
            nrows = self.nrows
            ncols = self.ncols
        else:
            nrows = 1
            ncols = 1

        ax = self.fig.add_subplot(nrows, ncols, iplt, aspect='equal')  # type: ignore

        xmin = 1e6
        xmax = -1e6
        ymin = 1e6
        ymax = -1e6
        for n in self.nodedict.keys():
            xx = self.nodedict[n]["x"]
            yy = self.nodedict[n]["y"]
            xmin = min(xmin, xx)
            xmax = max(xmax, xx)
            ymin = min(ymin, yy)
            ymax = max(ymax, yy)

        borderx = 0.1*(xmax-xmin)
        bordery = 0.1*(ymax-ymin)
        ax.set_xlim(xmin-borderx, xmax+borderx)
        ax.set_ylim(ymin-bordery, ymax+bordery)
        self.scale = (xmax-xmin) / 2
        ax.grid(True, which='both')
        for n in self.nodedict.keys():
            x = self.nodedict[n]["x"]
            y = self.nodedict[n]["y"]
            clr = self.GetNodeColor(n)
            ax.add_patch(patches.Circle((x, y), self.scale*0.07, color=clr))
            nodename = n.replace(".000000", "")  # remove trailing zeros if they are in the label
            ax.text(x, y, nodename,
                    fontsize=10, horizontalalignment='center', verticalalignment='center', color='white')

        # Now add the links between the nodes with cost
        for e in self.edgecost.keys():
            n1, n2 = e.split(":")
            x1 = self.nodedict[n1]["x"]
            y1 = self.nodedict[n1]["y"]
            x2 = self.nodedict[n2]["x"]
            y2 = self.nodedict[n2]["y"]
            ax.plot([x1, x2], [y1, y2],
                    color='green', linewidth=1)
            # now calculate the midpoint and add the cost
            xmid = (x1+x2)/2
            ymid = (y1+y2)/2
            ax.text(xmid, ymid, f"{self.edgecost[e]:.3f}",
                    fontsize=10, horizontalalignment='center', verticalalignment='center')

        plt.draw()

    def HightlightNodesInPath(self, path: list[str], cost, actionline: str):

        # Highlight the path we found
        ax = plt.gca()
        tit = "-".join(path) + f" cost:{cost:.3f}\n{actionline}"
        ax.set_title(tit)
        # ax.title = tit
        for n in path:
            x = self.nodedict[n]["x"]
            y = self.nodedict[n]["y"]
            ax.add_patch(patches.Circle((x, y), self.scale*0.07, edgecolor='red', lw=4, fill=False))

    fig = None
    iplot: int = 1
    nrows: int = 3
    ncols: int = 6

    def SetupPlot(self, tit: str):
        self.iplot = 1
        self.fig = plt.figure()
        self.fig.suptitle(tit)

    def ShowPlot(self):
        plt.show()

    def DoSubPlot(self, n: str, actionline: str, isSubPlot=True):
        self.PlotNodesWithNames(self.iplot, isSubPlot)
        curp = self.GetParentList(n)
        curp.reverse()
        cost = self.AstarCost(curp)
        self.iplot += 1
        self.HightlightNodesInPath(curp, cost, actionline)

    def AddNodeToOpenList(self, n: str):
        # global nodedict
        ntentcost = self.nodedict[n]["tent_tot_cost"]
        for i in range(len(self.openlist)):
            if ntentcost < self.nodedict[self.openlist[i]]["tent_tot_cost"]:
                self.openlist.insert(i, n)
                self.nodestat[n] = "open"
                # print(f"openlist:{openlist} {ntentcost}")
                return
        self.openlist.append(n)
        self.nodestat[n] = "open"
        # print(f"openlist:{openlist} {ntentcost}")

    def AddNodeToClosedList(self, n: str):
        self.closedlist.append(n)
        self.nodestat[n] = "closed"

    def GetParentList(self, n: str) -> list[str]:
        # global nodedict,parentnode
        rv = []
        while n in self.nodedict.keys():
            rv.append(n)
            if n not in self.parentnode.keys():
                break
            n = self.parentnode[n]
        return rv

    def GetSeeminglyClosestNodeToTarget(self) -> str:
        # global nodedict
        if len(self.openlist) == 1:
            return self.openlist[0]
        mincost = 1e6
        minnode: str = "None"
        for n in self.openlist:
            if "tent_tot_cost" not in self.nodedict[n].keys():
                continue
            ttcost = self.nodedict[n]["tent_tot_cost"]
            if ttcost < mincost:
                mincost = ttcost
                minnode = n
        return minnode

    def AssignParent(self, n: str, parent: str, goal: str):
        # global nodedict, parentnode
        self.parentnode[n] = parent
        pd = self.nodedict[parent]
        nd = self.nodedict[n]
        nd["cost"] = pd["cost"] + self.edgecost[f"{parent}:{n}"]
        nd["tent_tot_cost"] = nd["cost"] + self.Distance(n, goal)

    def FindPath(self, start: str, goal: str, scenename="Scene", stepplot=False, finplot=False) -> list[str]:
        # global nodedict, nbr, edgecost
        self.openlist = [start]
        self.closedlist = []
        if stepplot:
            finplot = False
        if start not in self.nodedict.keys():
            print(f"Error Start node ""{start}"" not in nodedict")
            return []
        if goal not in self.nodedict.keys():
            print(f"Error Goal node ""{goal}"" not in nodedict")
            return []

        if stepplot or finplot:
            self.SetupPlot(f"A* for {scenename} from {start} to {goal}")
        while len(self.openlist) > 0:
            n: str = self.GetSeeminglyClosestNodeToTarget()
            self.openlist.remove(n)
            if n == goal:
                rv = self.GetParentList(n)
                rv.reverse()
                if stepplot or finplot:
                    self.DoSubPlot(n, f"Solution", isSubPlot=not finplot)
                return rv
            for n2 in self.nbr[n]:
                if n2 in self.closedlist:
                    continue
                if n2 not in self.openlist:
                    self.AssignParent(n2, n, goal)
                    self.AddNodeToOpenList(n2)
                    if stepplot:
                        self.DoSubPlot(n2, f"added {n2} to openlist")
                else:
                    # if the nodes is in the openlist then we might need to reset the costs if we found a better path
                    if self.nodedict[n2]["cost"] > self.nodedict[n]["cost"] + self.edgecost[f"{n}:{n2}"]:
                        self.AssignParent(n2, n, goal)
            self.AddNodeToClosedList(n)
            if stepplot:
                self.DoSubPlot(n, f"added {n} to closed")

        return []

    def AstarCost(self, path) -> float:
        # global nodedict, edgecost
        cost = 0
        for i in range(len(path)-1):
            n1 = path[i]
            n2 = path[i+1]
            # print(self.edgecost[f"{n1}:{n2}"])
            cost += self.edgecost[f"{n1}:{n2}"]
        return cost

