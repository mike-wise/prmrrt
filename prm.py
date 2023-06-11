import astar
import argparse
import random
import math
import os
from shapely.geometry import LineString
from shapely.geometry import Point


parser = argparse.ArgumentParser(prog='PrmMain.py',
                                 description='Calculates PRM path around obstacles',
                                 epilog='Text at the bottom of help')

parser.add_argument('-n', '--nodes', type=str, default="nodes.csv",
                    help='Name of the nodes file')
parser.add_argument('-e', '--edges', type=str, default="edges.csv",
                    help='Name of the edges file')
parser.add_argument('-o', '--obstacles', type=str, default="obstacles.csv",
                    help='Name of the edges file')
parser.add_argument('-f', '--firstnode', type=str, default="1",
                    help='Name of the first node')
parser.add_argument('-t', '--targetnode', type=str, default="2",
                    help='Name of the target node')
parser.add_argument('-s', '--scene', type=str, default="PRM Planner",
                    help='Name of the scene')
parser.add_argument('-d', '--directory', type=str, default="planning_coursera",
                    help='Name of the directory')
parser.add_argument('-fp', '--finplot', action='store_true',
                    help='Do a plot of final path')
parser.add_argument('-sp', '--stepplot', action='store_true',
                    help='Create a plot that shows the steps to finding the final path')
parser.add_argument('-v', '--verbose', type=int, default=0,
                    help='Verbosity level')
parser.add_argument('-ns', '--noseed', action='store_true',
                    help='Do not use a random seed')
parser.add_argument('-seed', '--seed', type=int, default=1234,
                    help='Random seed value')
parser.add_argument('-n2g', '--nodes_to_gen', type=int, default=10,
                    help='PRM nodes to generate')


args = parser.parse_args()

fnamenodes = args.nodes
fnameedges = args.edges
fnameobstacles = args.obstacles
firstnode = args.firstnode
targetnode = args.targetnode
astarscene = args.scene
dname = args.directory
finplot = args.finplot
stepplot = args.stepplot
verbosity = args.verbose
no_seed = args.noseed
seed = args.seed
nodes_to_gen = args.nodes_to_gen


class PrmGen:

    nodedict: dict[str, dict[str, float]] = {}
    nodestat: dict[str, str] = {}
    parentnode: dict[str, str] = {}
    nbr: dict[str, list[str]] = {}
    edgecost: dict[str, float] = {}
    obst: list[dict[str, float]] = []

    def __init__(self, nodetxt: list[str], edgetxt: list[str], obsttxt: list[str] = None,
                 verbosity: int = 0, noseed: bool = False, seed: int = 1234):
        self.verbosity = verbosity
        if self.verbosity > 0:
            print(f"AStar.__init__")
            print("nodetxt:", nodetxt)
            print("edgetxt:", edgetxt)
            print("obsttxt:", obsttxt)
        if (len(nodetxt) == 1):
            nodetxt = self.FileToList(nodetxt[0])
        if (len(edgetxt) == 1):
            edgetxt = self.FileToList(edgetxt[0])
        if (obsttxt and len(obsttxt) == 1):
            obsttxt = self.FileToList(obsttxt[0])

        if self.verbosity > 0:
            print(f"There are {len(nodetxt)} nodes lines and {len(edgetxt)} edge lines")
            if obsttxt:
                print(f"There are {len(obsttxt)} obstacle lines")
            else:
                print(f"No obstacle file")

        for line in nodetxt:
            if line[0] == "#":
                continue
            if len(line) <= 1:
                continue
            n, xv, yv, cst = line.split(",")
            self.nodedict[n] = {"x": float(xv),
                                "y": float(yv),
                                "id": n,
                                "cost": float(cst),
                                "tent_tot_cost": float(cst)}
            self.nbr[n] = []
            self.nodestat[n] = "unvisited"

        if self.verbosity > 1:
            print("nodedict", self.nodedict)
            print("nbr", self.nbr)

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
                print(f'Error in edgelist: "{n1}" is not a node')
                continue
            if n2 not in self.nbr:
                print(f'Error in edgelist: "{n2}" is not a node')
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
        print(f"edge costs min:{mincost:.3f} max:{maxcost:.3f} avg:{sumcost/nedges:.3f}")

        if obsttxt:
            for line in obsttxt:
                if line[0] == "#":
                    continue
                if len(line) <= 1:
                    continue
                x, y, diam = line.split(",")
                self.obst.append({"x": float(x),
                                  "y": float(y),
                                  "diam": float(diam)})

        if not noseed:
            random.seed(seed)

        if verbosity > 0:
            if no_seed:
                print(f"Random seed not used") 
            else:
                print(f"Random seed set to {seed}")

        if self.verbosity > 1:
            print("obst", self.obst)

        if self.verbosity > 0:
            print(f"Prmgen has {len(self.nodedict)} nodes and {len(self.edgecost)} edges and {len(self.obst)} obstacles")

    def FileToList(self, fname: str) -> list[str]:
        if (os.path.isfile(fname)):
            with open(fname) as f:
                flist = f.readlines()
                return flist
        else:
            return []

    def GenNodesAndEdges(self, n: int, x0: float, y0: float, x1: float, y1: float):
        """
        Generate n additional nodes in the rectangle defined by x0,y0,x1,y1
        """
        org_node_ids = list(self.nodedict.keys())
        gen_node_ids = []
        startid = len(self.nodedict)+1
        for i in range(n):
            id = f"{startid+i}"
            x = random.uniform(x0, x1)
            y = random.uniform(y0, y1)
            self.nodedict[id] = {"x": x, "y": y, "id": id, "cost": 0, "tent_tot_cost": 0}
            self.nbr[id] = []
            self.nodestat[id] = "unvisited"
            gen_node_ids.append(id)

        for i1, id_i in enumerate(gen_node_ids):
            for j in range(i1+1, len(gen_node_ids)):
                id_j = gen_node_ids[j]
                if self.LineOfSight(id_i, id_j):
                    self.nbr[id_i].append(id_j)
                    self.nbr[id_j].append(id_i)
                    self.edgecost[f"{id_i}:{id_j}"] = self.Dist(id_i, id_j)
                    self.edgecost[f"{id_j}:{id_i}"] = self.Dist(id_j, id_i)

        for id_i in gen_node_ids:
            for id_j in org_node_ids:
                if self.LineOfSight(id_i, id_j):
                    self.nbr[id_i].append(id_j)
                    self.nbr[id_j].append(id_i)
                    self.edgecost[f"{id_i}:{id_j}"] = self.Dist(id_i, id_j)
                    self.edgecost[f"{id_j}:{id_i}"] = self.Dist(id_j, id_i)

    def LineOfSight(self, n1: str, n2: str) -> bool:
        """
        Check if the line between nodes n1 and n2 is clear of obstacles
        """
        x1 = self.nodedict[n1]["x"]
        y1 = self.nodedict[n1]["y"]
        x2 = self.nodedict[n2]["x"]
        y2 = self.nodedict[n2]["y"]
        for o in self.obst:
            xo = o["x"]
            yo = o["y"]
            diam = o["diam"]
            if self.LineCircleIntersect(x1, y1, x2, y2, xo, yo, diam):
                return False
        return True

    def LineCircleIntersect0(self, x1: float, y1: float, x2: float, y2: float, xo: float, yo: float, diam: float) -> bool:
        """
        Check if the line between (x1,y1) and (x2,y2) intersects the circle at (xo,yo) with diameter diam
        Obviously wrong since it isn't using the center of the circle idiot
        """
        # https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
        dx = x2 - x1
        dy = y2 - y1
        dr = math.sqrt(dx*dx + dy*dy)
        D = x1*y2 - x2*y1
        discriminant = diam*diam*dr*dr - D*D
        if discriminant < 0:
            return False
        else:
            return True

    def LineCircleIntersect(self, x1: float, y1: float, x2: float, y2: float, xo: float, yo: float, diam: float) -> bool:
        cenpt = Point(xo, yo)
        circ = cenpt.buffer(diam/2).boundary
        lineseg = LineString([(x1, y1), (x2, y2)])
        isect = circ.intersects(lineseg)
        # print(f"LineCircleIntersect: {isect}")
        return isect

    def Dist(self, n1: str, n2: str) -> float:
        """
        Return the distance between nodes n1 and n2
        """
        x1 = self.nodedict[n1]["x"]
        y1 = self.nodedict[n1]["y"]
        x2 = self.nodedict[n2]["x"]
        y2 = self.nodedict[n2]["y"]
        return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

    def ExtractNodesIntoList(self) -> list[str]:
        """
        Extract the nodes in the nodelist into a list of csv strings
        """
        rv = []
        for n in self.nodedict:
            x = self.nodedict[n]["x"]
            y = self.nodedict[n]["y"]
            rv.append(f"{n},{x:.3f},{y:.3f},100")
        return rv

    def ExtractEdgesIntoList(self) -> list[str]:
        """
        Extract the edges in the edgecost into a list of csv strings
        """
        rv = []
        for e in self.edgecost:
            n1, n2 = e.split(":")
            cost = self.edgecost[e]
            rv.append(f"{n1},{n2},{cost:.3f}")
        return rv


def main():
    fp_nodename = f"{dname}/{fnamenodes}"
    fp_edgename = f"{dname}/{fnameedges}"
    fp_obstacles = f"{dname}/{fnameobstacles}"

    prma = PrmGen([fp_nodename], [fp_edgename], [fp_obstacles], 
                  verbosity=verbosity, noseed=no_seed, seed=seed)
    prma.GenNodesAndEdges(nodes_to_gen, -0.5, -0.5, 0.5, 0.5)

    nodetxt = prma.ExtractNodesIntoList()
    edgetxt = prma.ExtractEdgesIntoList()

    asta = astar.AStar(nodetxt, edgetxt, [fp_obstacles], verbosity=verbosity)
    rv = asta.FindPath(firstnode, targetnode, scenename=astarscene,
                       stepplot=stepplot, finplot=finplot)
    print("bestpath:", rv)
    print(f"bestpath cost:{asta.AstarCost(rv):.5f}")
    asta.ShowPlot()

    # Write out the solution node path to "path.csv"
    nodestring = ",".join(rv)
    with open('path.csv', 'w') as file:
        file.write(nodestring)


if __name__ == "__main__":
    main()
