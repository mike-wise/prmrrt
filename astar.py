import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# f l a k e 8 : noqa



class AStar:

    nodedict: dict[str, dict[str, float]] = {}
    nodestat: dict[str, str] = {}
    parentnode: dict[str, str] = {}
    nbr: dict[str, list[str]] = {}
    edgecost: dict[str, float] = {}

    def __init__(self, nodetxt: list[str], edgetxt: list[str]):
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
        print("nodedict", self.nodedict)
        print("nbr", self.nbr)

        mincost = 1e6
        maxcost = -1e6
        sumcost = 0
        for line in edgetxt:
            if line[0] == "#":
                continue
            if len(line) <= 1:
                continue
            n1, n2, cost = line.split(",")
            self.nbr[n1].append(n2)
            self.nbr[n2].append(n1)
            if float(cost) < mincost:
                mincost = float(cost)
            if float(cost) > maxcost:
                maxcost = float(cost)
            sumcost += float(cost)
            self.edgecost[f"{n1}:{n2}"] = float(cost)
            self.edgecost[f"{n2}:{n1}"] = float(cost)
        print(f"mincost:{mincost:.3f} maxcost:{maxcost:.3f} avgcost:{sumcost/len(self.edgecost):.3f}")
        # print("edgecost",edgecost)

    def Distance(self, n1: str, n2: str):
        x1 = self.nodedict[n1]["x"]
        y1 = self.nodedict[n1]["y"]
        x2 = self.nodedict[n2]["x"]
        y2 = self.nodedict[n2]["y"]
        return numpy.sqrt((x1-x2)**2 + (y1-y2)**2)

    def CheckDistances(self):
        for e in self.edgecost.keys():
            n1, n2 = e.split(":")
            dist = self.Distance(n1, n2)
            if dist > self.edgecost[e]:
                m = f"CheckDist error dist>edgcost: {n1} {n2} {dist:.1f} {self.edgecost[e]}"
                print(m)

    def GetNodeColor(self, n: str):
        # Named colors: https://matplotlib.org/stable/gallery/color/named_colors.html
        if self.nodestat[n] == "open":
            return "lightseagreen"
        if self.nodestat[n] == "closed":
            return "darkgreen"
        if self.nodestat[n] == "unvisited":
            return "blue"
        return "black"

    def PlotNodesWithNames(self, iplt: int):

        # Plot the nodes using matplotlib

        nrows = self.nrows
        ncols = self.ncols

        ax = self.fig.add_subplot(nrows, ncols, iplt, aspect='equal')  # type: ignore

        xmin = 1e6
        xmax = -1e6
        ymin = 1e6
        ymax = -1e6
        for n in self.nodedict.keys():
            xx = self.nodedict[n]["x"]
            yy = self.nodedict[n]["y"]
            if xx < xmin:
                xmin = xx
            if self.nodedict[n]["x"] > xmax:
                xmax = xx
            if yy < ymin:
                ymin = yy
            if yy > ymax:
                ymax = yy

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

    def setupPlot(self, tit: str):
        # global nodedict, iplot, nrows, ncols, fig
        self.iplot = 1
        self.fig = plt.figure()
        self.fig.suptitle(tit)

    def doSubPlot(self, n: str, actionline: str):
        # global nodedict, iplot, nrows, ncols
        self.PlotNodesWithNames(self.iplot)
        curp = self.getParentList(n)
        curp.reverse()
        cost = self.astarcost(curp)
        self.iplot += 1
        self.HightlightNodesInPath(curp, cost, actionline)

    def addNodeToOpenList(self, n: str):
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

    def addNodeToClosedList(self, n: str):
        self.closedlist.append(n)
        self.nodestat[n] = "closed"

    def getParentList(self, n: str) -> list[str]:
        # global nodedict,parentnode
        rv = []
        while n in self.nodedict.keys():
            rv.append(n)
            if n not in self.parentnode.keys():
                break
            n = self.parentnode[n]
        return rv

    def getSeeminglyClosestNodeToTarget(self) -> str:
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

    def assignParent(self, n: str, parent: str, goal: str):
        # global nodedict, parentnode
        self.parentnode[n] = parent
        pd = self.nodedict[parent]
        nd = self.nodedict[n]
        nd["cost"] = pd["cost"] + self.edgecost[f"{parent}:{n}"]
        nd["tent_tot_cost"] = nd["cost"] + self.Distance(n, goal)

    def astar(self, start: str, goal: str) -> list[str]:
        # global nodedict, nbr, edgecost
        self.openlist = [start]
        self.closedlist = []
        self.setupPlot(f"A* for astarscene")
        while len(self.openlist) > 0:
            n: str = self.getSeeminglyClosestNodeToTarget()
            self.openlist.remove(n)
            if n == goal:
                rv = self.getParentList(n)
                rv.reverse()
                self.doSubPlot(n, f"Solution")
                return rv
            for n2 in self.nbr[n]:
                if n2 in self.closedlist:
                    continue
                if n2 not in self.openlist:
                    self.assignParent(n2, n, goal)
                    self.addNodeToOpenList(n2)
                    self.doSubPlot(n2, f"added {n2} to openlist")
                else:
                    # if the nodes is in the openlist then we might need to reset the costs if we found a better path
                    if self.nodedict[n2]["cost"] > self.nodedict[n]["cost"] + self.edgecost[f"{n}:{n2}"]:
                        self.assignParent(n2, n, goal)
            self.addNodeToClosedList(n)
            self.doSubPlot(n, f"added {n} to closed")

        return []

    def astarcost(self, path) -> float:
        # global nodedict, edgecost
        cost = 0
        for i in range(len(path)-1):
            n1 = path[i]
            n2 = path[i+1]
            print(self.edgecost[f"{n1}:{n2}"])
            cost += self.edgecost[f"{n1}:{n2}"]
        return cost

    def pltshow():
        plt.show()
