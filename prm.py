import astar
import random
import math
import os


class PrmGen(astar.AStar):
    """
    Class to generate PRM nodes and edges
    """

    seed = 1234

    def __init__(self, nodetxt: list[str], edgetxt: list[str], obsttxt: list[str] = None,
                 verbosity: int = 1, ranseed: bool = False, seed: int = 1234):
        """
        Constructor
        :param nodetxt: list of node lines
        :param edgetxt: list of edge lines
        :param obsttxt: list of obstacle lines
        :param verbosity: verbosity level
        :param seed: seed value
        """
        super().__init__(nodetxt, edgetxt, obsttxt, verbosity)

        if ranseed:
            self.seed = random.randint(0, 10000)
            if verbosity > 0:
                print("Generated random seed:", self.seed)
        else:
            self.seed = seed

        random.seed(self.seed)
        print(f"Random seed set to {self.seed}")

        self.maxlinks = 3

        if self.verbosity > 1:
            print("obst", self.obst)

        if self.verbosity > 0:
            print(f"Prmgen nodes:{len(self.nodedict)} edges:{len(self.edgecost)} obstacles:{len(self.obst)}")

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
        startid = len(self.nodedict)+1
        # startid = len(self.nodedict)+1
        while len(gen_node_ids) < n:
            i = len(gen_node_ids)
            # id = f"{startid+i}"
            id = f"{startid+i}"
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

        for i, id_i in enumerate(org_node_ids):
            todo = gen_node_ids.copy()
            self.TryConnectListClosestK(id_i, todo, maxlinks=self.maxlinks)

        for i, id_i in enumerate(gen_node_ids):
            todo = org_node_ids.copy()
            todo.extend(gen_node_ids[i+1:])
            self.TryConnectListClosestK(id_i, todo, maxlinks=self.maxlinks)

        if self.verbosity > 1:
            print("org_node_ids:", org_node_ids)
            print("gen_node_ids:", gen_node_ids)

    def TryConnectListClosestK(self, id_i: str, id_list: list[str], maxlinks: int = 3) -> bool:
        """
        Try to connect node id_i to the closest nodes in id_list
        """
        nlinks = 0
        linklst = []
        ncand = len(id_list)
        while len(id_list) > 0:
            id_j = self.FindClosestNodeInList(id_i, id_list)
            id_list.remove(id_j)
            if self.TryConnect(id_i, id_j):
                nlinks += 1
                linklst.append(id_j)
                if nlinks >= maxlinks:
                    if self.verbosity > 1:
                        print(f"Connected node {id_i} to {nlinks}/{ncand} nodes:{linklst}")
                        return True

        if self.verbosity > 1:
            print(f"Connected node {id_i} to {nlinks}/{ncand} nodes out of {len(id_list)}:{linklst}")
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
        # print(f"LineOfSight: {n1} -> {n2}")
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
                if self.verbosity > 1:
                    print(f"LineOfSight: {n1} -> {n2} intersects obstacle {o['id']}")
                return False
        return True

    def LineCircleIntersect(self, x1, y1, x2, y2, cx, cy, diam) -> bool: 
        """
        Check if the line between (x1,y1) and (x2,y2) intersects the circle at (xo,yo) with diameter diam
        """
        # https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm

        rad = diam/2

        dx = x2 - x1
        dy = y2 - y1
        a = dx*dx + dy*dy
        t = (dx*(cx-x1) + dy*(cy-y1)) / a
        if t < 0:
            return False
        if t > 1:
            return False
        x = x1 + t*dx
        y = y1 + t*dy
        dist = math.sqrt((x-cx)**2 + (y-cy)**2)
        if dist > rad:
            return False
        return True

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
            rv.append(f"{n},{x:.3f},{y:.3f},100\n")
        return rv

    def ExtractEdgesIntoList(self) -> list[str]:
        """
        Extract the edges in the edgecost into a list of csv strings
        """
        rv = []
        for e in self.edgecost:
            n1, n2 = e.split(":")
            cost = self.edgecost[e]
            rv.append(f"{n1},{n2},{cost:.3f}\n")
        return rv

    def ExtractObstIntoList(self) -> list[str]:
        """
        Extract the obstacles in the obst into a list of csv strings
        """
        rv = []
        for o in self.obst:
            x = o["x"]
            y = o["y"]
            diam = o["diam"]
            rv.append(f"{x:.3f},{y:.3f},{diam:.3f}\n")
        return rv


