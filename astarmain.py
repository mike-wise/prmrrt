import astar
import argparse

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

parser = argparse.ArgumentParser(
                    prog='AstarMain',
                    description='Calculates Astar path for a set of nodes and edges',
                    epilog='Text at the bottom of help')

parser.add_argument('-n', '--nodes', type=str, default="nodes.csv",
                    help='Name of the nodes file')
parser.add_argument('-e', '--edges', type=str, default="edges.csv",
                    help='Name of the edges file')
parser.add_argument('-f', '--firstnode', type=str, default="1",
                    help='Name of the first node')
parser.add_argument('-t', '--targetnode', type=str, default="12",
                    help='Name of the target node')
parser.add_argument('-s', '--scene', type=str, default="Scene5",
                    help='Name of the scene')
parser.add_argument('-d', '--directory', type=str, default="Scene5",
                    help='Name of the directory')
parser.add_argument('-fp', '--finplot', action='store_true',
                    help='Do a plot of final path')
parser.add_argument('-sp', '--stepplot', action='store_true', 
                    help='Create a plot that shows the steps to finding the final path')


args = parser.parse_args()

fnamenodes = args.nodes
fnameedges = args.edges
firstnode = args.firstnode
targetnode = args.targetnode
astarscene = args.scene
dname = args.directory
finplot = args.finplot
stepplot = args.stepplot


# dname = "planning_coursera"

# astarscene = "PRM2"
# astarscene = "Scene5"
# astarscene = "PRM"
# astarscene = ""
# astarscene = "Scene5"


# if astarscene == "Scene5":
#     dname = "Scene5"
#     fnamenodes = "nodes.csv"
#     fnameedges = "edges.csv"
#     firstnode = "1"
#     targetnode = "12"

# elif astarscene == "oldPRM":
#     dname = "oldPRM"
#     fnamenodes = "nodes.csv"
#     fnameedges = "edges.csv"
#     firstnode = "1.000000"
#     targetnode = "36.000000"

# elif astarscene == "PRM2":
#     dname = "oldPRM"
#     fnamenodes = "nodes2.csv"
#     fnameedges = "PRM/edges2.csv"
#     firstnode = "1.000000"
#     targetnode = "36.000000"

# if fnamenodes != "":
#     with open(f"{dname}/{fnamenodes}") as file:
#         sizehint = 0
#         nodetxt = file.readlines(sizehint)

#     print(f"There are {len(nodetxt)} nodes")
#     # print(nodetxt)

# if fnameedges != "":
#     with open(f"{dname}/{fnameedges}") as file:
#         sizehint = 0
#         edgetxt = file.readlines(sizehint)

#     print(f"There are {len(edgetxt)} edges")


def main():
    fp_nodename = f"{dname}/{fnamenodes}"
    fp_edgename = f"{dname}/{fnameedges}"
    asta = astar.AStar([fp_nodename], [fp_edgename])
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
