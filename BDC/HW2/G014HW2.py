from pyspark import SparkContext, SparkConf
import sys
import os
import random as rand
from collections import defaultdict
from operator import add
import time
import statistics as stat


def CountTriangles(edges):
    # Create a defaultdict to store the neighbors of each vertex
    neighbors = defaultdict(set)
    for edge in edges:
        u, v = edge
        neighbors[u].add(v)
        neighbors[v].add(u)

    # Initialize the triangle count to zero
    triangle_count = 0

    # Iterate over each vertex in the graph.
    # To avoid duplicates, we count a triangle <u, v, w> only if u<v<w
    for u in neighbors:
        # Iterate over each pair of neighbors of u
        for v in neighbors[u]:
            if v > u:
                for w in neighbors[v]:
                    # If w is also a neighbor of u, then we have a triangle
                    if w > v and w in neighbors[u]:
                        triangle_count += 1
    # Return the total number of triangles in the graph
    return triangle_count

def countTriangles2(colors_tuple, edges, rand_a, rand_b, p, num_colors):
    #We assume colors_tuple to be already sorted by increasing colors. Just transform in a list for simplicity
    colors = list(colors_tuple)
    #Create a dictionary for adjacency list
    neighbors = defaultdict(set)
    #Creare a dictionary for storing node colors
    node_colors = dict()
    for edge in edges:

        u, v = edge
        node_colors[u]= ((rand_a*u+rand_b)%p)%num_colors
        node_colors[v]= ((rand_a*v+rand_b)%p)%num_colors
        neighbors[u].add(v)
        neighbors[v].add(u)

    # Initialize the triangle count to zero
    triangle_count = 0

    # Iterate over each vertex in the graph
    for v in neighbors:
        # Iterate over each pair of neighbors of v
        for u in neighbors[v]:
            if u > v:
                for w in neighbors[u]:
                    # If w is also a neighbor of v, then we have a triangle
                    if w > u and w in neighbors[v]:
                        # Sort colors by increasing values
                        triangle_colors = sorted((node_colors[u], node_colors[v], node_colors[w]))
                        # If triangle has the right colors, count it.
                        if colors==triangle_colors:
                            triangle_count += 1
    # Return the total number of triangles in the graph
    return triangle_count


# hash function
def hc(a, b, p, C, u):
    return ((a * u + b) % p) % C


# ALGORITHM 1
def MR_ApproxTCwithNodeColors(edges, C):
    p = 8191
    a = rand.randint(1,p-1)
    b = rand.randint(0,p-1)

    # coloring edges
    def edge_coloring(edge):
        u, v = edge
        i = hc(a,b,p,C,u)
        if(i == hc(a,b,p,C,v)):
            return i
        else: # returns -1 if the edge have different color verteces
            return -1

    triangles_count = (edges.map(lambda x: (edge_coloring(x), x)).filter(lambda x: x[0]>-1)    # MAP PHASE (R1)
                        .groupByKey()   # SHUFFLE+GROUPING
                        .map(lambda x: (0, CountTriangles(x[1])))   # REDUCE PHASE (R1)
                        .reduceByKey(add))  # REDUCE PHASE (R2)
    return (triangles_count.values().collect()[0])*pow(C,2)


# create C key-values pairs
def create_pairs(x, p,a,b,C):
    pairs = list()
    h1 = hc(a,b,p,C,x[0])
    h2 = hc(a,b,p,C,x[1])
    for i in range(C):
        key = [h1, h2, i]
        key.sort()
        pairs.append( (tuple(key), x) )
    return pairs

# ALGORITHM 2
def MR_ExactTC(edges, C):
    p = 8191
    a = rand.randint(1,p-1)
    b = rand.randint(0,p-1)

    triangles_count = (edges.flatMap(lambda x: create_pairs(x,p,a,b,C) )   # MAP PHASE (R1)
                        .groupByKey()   # SHUFFLE+GROUPING
                        .map(lambda x: (0, countTriangles2(x[0],x[1],a,b,p,C)) )    # REDUCE PHASE (R1)
                        .reduceByKey(add)   # REDUCE PHASE (R2)
                    )

    return triangles_count.collect()[0][1]



def main():
    # CHECKING NUMBER OF CMD LINE PARAMTERS
    assert len(sys.argv) == 5, "Usage: python G014HW1.py <C> <R> <F> <file_name>"

    # SPARK SETUP
    conf = SparkConf().setAppName('G014HW2')
    conf.set("spark.locality.wait", "0s")
    sc = SparkContext(conf=conf)

    # INPUT READING

	# 1. Read number of partitions
    C = sys.argv[1]
    assert C.isdigit(), "C must be an integer"
    C = int(C)

    # 2. Read number of rounds
    R = sys.argv[2]
    assert R.isdigit(), "R must be an integer"
    R = int(R)

    # 3. Read binary flag
    F = sys.argv[3]
    assert F.isdigit(), "F must be an integer either 0 or 1"
    F = int(F)
    assert ( F == 0 or F==1), "F must be 0 or 1"

	# 4. Read input file and subdivide it into C random partitions
    data_path = sys.argv[4]
    # the following command is commented for running the script on the cluster
    #assert os.path.isfile(data_path), "File or folder not found"
    numPartitions = 32
    rawData = sc.textFile(data_path, minPartitions=numPartitions).cache()
    edges = rawData.map(lambda x: (int(x.split(",")[0]),int(x.split(",")[1]))).repartition(numPartitions=numPartitions).cache()

    # printing
    print("Dataset =", data_path)
    print("Number of Edges =", edges.count())
    print("Number of Colors =", C)
    print("Number of Repetitions =", R)

    ex_time_list = []   # list for saving the running time of each repetition
    t_final_list = []   # list for saving the number of triangles returned by the ALGORITHM at each repetition


    # if true run ALGORITHM 1 R times
    if(F == 0):
        for i in range(R):
            st = time.time()
            t_final_list.append(MR_ApproxTCwithNodeColors(edges,C))
            et = time.time()
            elapsed_time = et - st
            ex_time_list.append(elapsed_time)

        median_t_final = stat.median(t_final_list)
        average_execution = stat.mean(ex_time_list)
        print("Approximation algorithm with node coloring")
        print("- Number of triangles (median over",R,"runs) =",median_t_final)
        print("- Running time (average over",R,"runs) =",round(average_execution*1000),"ms")


    # if true run ALGORITHM 2 R times
    if( F == 1 ):
        for i in range(R):
            st = time.time()
            t_final_list.append(MR_ExactTC(edges, C))
            et = time.time()
            elapsed_time = et - st
            ex_time_list.append(elapsed_time)

        average_execution = stat.mean(ex_time_list)
        print ("Exact algorithm with node coloring")
        print("- Number of triangles =", t_final_list[-1])
        print("- Running time (average over",R,"runs) =",round(average_execution*1000),"ms")


if __name__ == "__main__":
    main()
