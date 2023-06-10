import astar

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

fnamenodes = "nodes.csv"
fnameedges = "edges.csv"
firstnode = "1"
targetnode = "2"

dname = "planning_coursera"


def main():
    a = astar.AStar(nodetxt, edgetxt)
    rv = a.astar('1', '6')
    print("bestpath:", rv)
    print(f"{a.astarcost(rv):.5f}")
    a.pltshow()
    altpath = ['1', '2', '5', '7', '10', '12']
    print("altpath:", altpath)
    print(f"{a.astarcost(altpath):.5f}")
    altpath1 = ['1', '3', '4', '8', '12']
    print("altpath1:", altpath1)
    print(f"{a.astarcost(altpath1):.5f}")
    altpath3 = ['1', '2', '5', '7', '10', '12']
    print("altpath3:", altpath3)
    print(f"{a.astarcost(altpath3):.5f}")

    # Write out the solution node path to "path.csv"
    nodestring = ",".join(rv)
    with open('path.csv', 'w') as file:
        file.write(nodestring)


if __name__ == "__main__":
    main()
