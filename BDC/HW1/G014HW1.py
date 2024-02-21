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

# ALGORITHM 1
def MR_ApproxTCwithNodeColors(edges, C):
    # hash function
    p = 8191
    a = rand.randint(1,p-1)
    b = rand.randint(0,p-1)
    def hc(a, b, p, C, u):
        return ((a * u) % p) % C
    
    # coloring edges
    def edge_coloring(edge):
        u, v = edge
        i = hc(a,b,p,C,u)
        if(i == hc(a,b,p,C,v)):
            return i
        else: # returns -1 if the edge have different color verteces
            return -1
    
    triangles_count = (edges.map(lambda x: (edge_coloring(x), x)).filter(lambda x: x[0]>-1)     # MAP PHASE (R1)
                        .groupByKey()   # SHUFFLE+GROUPING
                        .map(lambda x: (0, CountTriangles(x[1])))   # REDUCE PHASE (R1)
                        .reduceByKey(add)) # REDUCE PHASE (R2)
    return (triangles_count.values().collect()[0])*pow(C,2)

def gather_triangles_partitions(pairs):
    edges = []
    for p in pairs:
        edges.append(p)
    return [(0, CountTriangles(edges))]

# ALGORITHM 2
def MR_ApproxTCwithSparkPartitions(edges, C):
    # MAP PHASE (R1) already done in main section
    triangles_count = (edges.mapPartitions(gather_triangles_partitions) # REDUCE PHASE (R1)
                        .reduceByKey(add))  # REDUCE PHASE (R2)
    return (triangles_count.values().collect()[0])*pow(C,2)

def main():
    # CHECKING NUMBER OF CMD LINE PARAMTERS
    assert len(sys.argv) == 4, "Usage: python G014HW1.py <C> <R> <file_name>"
    
    # SPARK SETUP
    conf = SparkConf().setAppName('G014HW1')
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

	# 3. Read input file and subdivide it into C random partitions
    data_path = sys.argv[3]
    assert os.path.isfile(data_path), "File or folder not found"
    rawData = sc.textFile(data_path, minPartitions=C).cache()
    edges = rawData.map(lambda x: (int(x.split(",")[0]),int(x.split(",")[1]))).repartition(numPartitions=C).cache()

    # printing
    print("Dataset =", data_path)
    print("Number of Edges =", edges.count())
    print("Number of Colors =", C)
    print("Number of Repetitions =", R)

    ex_time_list = []
    t_final_list = []

    for i in range(R):
        st = time.time()
        t_final_list.append(MR_ApproxTCwithNodeColors(edges,C))
        et = time.time()
        elapsed_time = et - st
        ex_time_list.append(elapsed_time)

    median_t_final = stat.median(t_final_list)
    average_execution = stat.mean(ex_time_list)
    print("Approximation through node coloring")
    print("- Number of triangles (median over",R,"runs) =",median_t_final)
    print("- Running time (average over",R,"runs) =",round(average_execution*1000),"ms")
    print ("Approximation throught Spark partitioning")
    st = time.time()
    t_final = MR_ApproxTCwithSparkPartitions(edges,C)
    et = time.time()
    print("- Number of triangles =",t_final)
    print("- Running time =",round((et-st)*1000),"ms")
    

if __name__ == "__main__":
    main()