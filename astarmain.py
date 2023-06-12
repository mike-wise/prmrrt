import astar
import argparse

parser = argparse.ArgumentParser(prog='AstarMain',
                                 description='Calculates Astar path for a set of nodes and edges',
                                 epilog='Text at the bottom of help')

parser.add_argument('-n', '--nodes', type=str, default="nodes.csv",
                    help='Name of the nodes file')
parser.add_argument('-e', '--edges', type=str, default="edges.csv",
                    help='Name of the edges file')
parser.add_argument('-of', '--obstacles', type=str, default="obstacles.csv",
                    help='Name of the obstacles file')
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
parser.add_argument('-v', '--verbose', type=int, default=0,
                    help='Verbosity level')


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

if verbosity > 0:
    print("prm.py args:")
    print("    fnamenodes:", fnamenodes)
    print("    fnameedges:", fnameedges)
    print("    fnameobstacles:", fnameobstacles)
    print("    firstnode:", firstnode)
    print("    targetnode:", targetnode)
    print("    astarscene:", astarscene)
    print("    dname:", dname)
    print("    finplot:", finplot)
    print("    stepplot:", stepplot)
    print("    verbosity:", verbosity)


def main():
    fp_nodename = f"{dname}/{fnamenodes}"
    fp_edgename = f"{dname}/{fnameedges}"
    fp_obstename = f"{dname}/{fnameobstacles}"
    asta = astar.AStar([fp_nodename], [fp_edgename], [fp_obstename], verbosity=verbosity)
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
