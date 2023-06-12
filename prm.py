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

parser.add_argument('-nf', '--nodes', type=str, default="nodes.csv",
                    help='Name of the nodes file')
parser.add_argument('-ef', '--edges', type=str, default="edges.csv",
                    help='Name of the edges file')
parser.add_argument('-of', '--obstacles', type=str, default="obstacles.csv",
                    help='Name of the obstacles file')
parser.add_argument('-fn', '--firstnode', type=str, default="1",
                    help='Name of the first node')
parser.add_argument('-tn', '--targetnode', type=str, default="2",
                    help='Name of the target node')
parser.add_argument('-n2g', '--nodes_to_gen', type=int, default=10,
                    help='PRM nodes to generate')
parser.add_argument('-mxl', '--maxlinks', type=int, default=3,
                    help='Max Links to add to a PRM node')
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


args = parser.parse_args()

fnamenodes = args.nodes
fnameedges = args.edges
fnameobstacles = args.obstacles
firstnode = args.firstnode
targetnode = args.targetnode
maxlinks = args.maxlinks
astarscene = args.scene
dname = args.directory
finplot = args.finplot
stepplot = args.stepplot
verbosity = args.verbose
no_seed = args.noseed
seed = args.seed
nodes_to_gen = args.nodes_to_gen

if verbosity > 0:
    print("prm.py args:")
    print("    fnamenodes:", fnamenodes)
    print("    fnameedges:", fnameedges)
    print("    fnameobstacles:", fnameobstacles)
    print("    firstnode:", firstnode)
    print("    targetnode:", targetnode)

    print("    maxlinks:", maxlinks)
    print("    nodes_to_gen:", nodes_to_gen)

    print("    astarscene:", astarscene)
    print("    dname:", dname)
    print("    finplot:", finplot)
    print("    stepplot:", stepplot)
    print("    verbosity:", verbosity)
    print("    no_seed:", no_seed)
    print("    seed:", seed)


class PrmGen:
    """
    Class to generate PRM nodes and edges
    """

    nodedict: dict[str, dict[str, float]] = {}
    nodestat: dict[str, str] = {}
    parentnode: dict[str, str] = {}
    nbr: dict[str, list[str]] = {}
    edgecost: dict[str, float] = {}
    obst: list[dict[str, float]] = []

    def __init__(self, nodetxt: list[str], edgetxt: list[str], obsttxt: list[str] = None,
                 verbosity: int = 1, noseed: bool = False, seed: int = 1234):
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
                id = len(self.obst)
                self.obst.append({"id":str(id),
                                  "x": float(x),
                                  "y": float(y),
                                  "diam": float(diam)})

        if not noseed:
            random.seed(seed)

        if verbosity > 0:
            if no_seed:
                print(f"Random seed not used")
            else:
                print(f"Random seed set to {seed}")

        self.maxlinks = 3

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

    def GenNodesAndEdges(self, n: int, x0: float, y0: float, x1: float, y1: float, maxlinks: int = 3):
        """
        Generate n additional nodes in the rectangle defined by x0,y0,x1,y1
        """
        self.maxlinks = maxlinks
        org_node_ids = list(self.nodedict.keys())
        gen_node_ids = []
        # startid = len(self.nodedict)+1
        while len(gen_node_ids) < n:
            i = len(gen_node_ids)
            # id = f"{startid+i}"
            id = f"_{i}"
            x = random.uniform(x0, x1)
            y = random.uniform(y0, y1)

            # Don't generate nodes inside obstacles
            isclear = True
            for o in self.obst:
                d2o = self.DistToObst(x, y, o)
                if d2o < o["diam"]:
                    if self.verbosity > 3:
                        print(f"Candidate Node {id} x:{x:.3f} y:{y:.3f}  is inside obstacle {o} dist:{d2o:.3f}")
                    isclear = False
                    break
            if not isclear:
                continue

            self.nodedict[id] = {"x": x, "y": y, "id": id, "cost": 0, "tent_tot_cost": 0}
            self.nbr[id] = []
            self.nodestat[id] = "unvisited"
            if self.verbosity > 3:
                print(f"Generated node {id} x:{x:.3f} y:{y:.3f}")
            gen_node_ids.append(id)

        for i, id_i in enumerate(gen_node_ids):
            todo = org_node_ids.copy()
            todo.extend(gen_node_ids[i+1:])
            self.TryConnectListClosestK(id_i, todo, maxlinks=self.maxlinks)

        # for id_i in gen_node_ids:
        #     for id_j in org_node_ids:
        #         self.TryConnect(id_i, id_j)

        # for i1, id_i in enumerate(gen_node_ids):
        #     for j in range(i1+1, len(gen_node_ids)):
        #         id_j = gen_node_ids[j]
        #         self.TryConnect(id_i, id_j)

        print("org_node_ids:", org_node_ids)
        print("gen_node_ids:", gen_node_ids)

    def TryConnectListClosestK(self, id_i: str, id_list: list[str], maxlinks: int = 2) -> bool:
        """
        Try to connect node id_i to the closest nodes in id_list
        """
        nlinks = 0
        while len(id_list) > 0:
            id_j = self.FindClosestNodeInList(id_i, id_list)
            id_list.remove(id_j)
            if self.TryConnect(id_i, id_j):
                nlinks += 1
                if nlinks >= maxlinks:
                    return True

        return False

    def FindClosestNodeInList(self, id: str, id_list: list[str]) -> str:
        """
        Find the node in id_list that is closest to node id
        """
        mindist = 1e9
        minid = ""
        # brute force search for now
        # a more sophisticated search would
        # - set up lists sorting points according to their position per dimension before this is called
        # - and then use a binary search to find the closest point
        for id_j in id_list:
            dist = self.Dist(id, id_j)
            if dist < mindist:
                mindist = dist
                minid = id_j
        return minid

    def TryConnect(self, id_i: str, id_j: str) -> bool:
        """
        Try to connect nodes id_i and id_j
        """
        if id_i == id_j:
            print(f"TryConnect Warning: tried to connect node to itself: {id_i}")
            return False
        if self.LineOfSight(id_i, id_j):
            edgeid1 = f"{id_i}:{id_j}"
            if edgeid1 in self.edgecost:
                return True
            self.nbr[id_i].append(id_j)
            self.nbr[id_j].append(id_i)
            self.edgecost[f"{id_i}:{id_j}"] = self.Dist(id_i, id_j)
            self.edgecost[f"{id_j}:{id_i}"] = self.Dist(id_j, id_i)
            return True
        else:
            return False

    def DistToObst(self, x: float, y: float, o: dict) -> float:
        """
        Distance from point (x,y) to center of obstacle o
        """
        xo = o["x"]
        yo = o["y"]
        return math.sqrt((x-xo)**2 + (y-yo)**2)

    def LineOfSight(self, n1: str, n2: str) -> bool:
        """
        Check if the line between nodes n1 and n2 is clear of obstacles
        """
        nn1 = self.nodedict[n1]
        nn2 = self.nodedict[n2]
        x1 = nn1["x"]
        y1 = nn1["y"]
        x2 = nn2["x"]
        y2 = nn2["y"]
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
        Co-pilot obviously got this very wrong since it isn't using the center of the circle
        Don't use this obviously, leaving it in for my own amusement
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
        nn1 = self.nodedict[n1]
        nn2 = self.nodedict[n2]
        x1 = nn1["x"]
        y1 = nn1["y"]
        x2 = nn2["x"]
        y2 = nn2["y"]
        return math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

    def ExtractNodesIntoList(self) -> list[str]:
        """
        Extract the nodes in the nodelist into a list of csv strings
        """
        rv = []
        for n in self.nodedict:
            nn = self.nodedict[n]
            x = nn["x"]
            y = nn["y"]
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
    prma.GenNodesAndEdges(nodes_to_gen, -0.5, -0.5, 0.5, 0.5, maxlinks)

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
