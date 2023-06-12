import prm
import argparse


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
parser.add_argument('-rs', '--ranseed', action='store_true',
                    help='Generate a random seed and print it out')
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
ran_seed = args.ranseed
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
    print("    ran_seed:", ran_seed)
    print("    seed:", seed)


def main():
    fp_nodename = f"{dname}/{fnamenodes}"
    fp_edgename = f"{dname}/{fnameedges}"
    fp_obstacles = f"{dname}/{fnameobstacles}"

    prma = prm.PrmGen([fp_nodename], [fp_edgename], [fp_obstacles], 
                      verbosity=verbosity, ranseed=ran_seed, seed=seed)
    prma.GenNodesAndEdges(nodes_to_gen, -0.5, -0.5, 0.5, 0.5, maxlinks)

    nodelist = prma.ExtractNodesIntoList()
    edgelist = prma.ExtractEdgesIntoList()
    obstlist = prma.ExtractObstIntoList()

    rv = prma.FindPath(firstnode, targetnode, scenename=astarscene,
                       stepplot=stepplot, finplot=finplot)
    print("bestpath:", rv)
    print(f"bestpath cost:{prma.AstarCost(rv):.5f}")
    prma.ShowPlot()

    # Write out the solution node path to "path.csv"
    pathline = ",".join(rv)
    with open('path.csv', 'w') as file:
        file.write(pathline)

    # Write out generated edges and nodes
    with open('nodes.csv', 'w') as file:
        file.writelines(nodelist)

    with open('edges.csv', 'w') as file:
        file.writelines(edgelist)

    with open('obstacles.csv', 'w') as file:
        file.writelines(obstlist)


if __name__ == "__main__":
    main()
