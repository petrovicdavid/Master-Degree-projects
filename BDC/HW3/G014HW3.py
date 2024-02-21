from pyspark import SparkContext, SparkConf
from pyspark.streaming import StreamingContext
from pyspark import StorageLevel
import threading
import sys
import statistics as stat
import random as rand

# Maximum (approximately) stream length
THRESHOLD = 10000000

# Hash function
def hc(a, b, p, C, u):
    return ((a * u + b) % p) % C

# Retrieve the most frequent K items and the corresponding frequencies
def get_first_k(dictionary, k):
    dictionary_sorted = sorted(dictionary.items(), key=lambda x: x[1], reverse=True)
    new_dictionary = dict(dictionary_sorted[:k])
    return new_dictionary


# Operations to perform after receiving an RDD 'batch'
def process_batch(batch):
    global totalStreamLength, histogram, streamLengthFiltered
    global D, W, R, C, hash_parameters_h, hash_parameters_g, p

    # Total number of items in the batch
    total_batch_size = batch.count()

    # Return if the total number of items received is greater then the threshold
    if totalStreamLength[0]>=THRESHOLD:
        return

    # Update the total stream length
    totalStreamLength[0] += total_batch_size

    # Filtering the items to obtain the ones in the specified range
    batch_filtered = batch.filter(lambda x: int(x)>=R[0] and int(x)<=R[1])

    # Update the length of the filtered stream
    streamLengthFiltered[0] += batch_filtered.count()


    # Extract the exact frequencies of items from the batch
    batch_items = batch_filtered.map(lambda s: (int(s), 1)).reduceByKey(lambda i1, i2: i1+i2).collect()

    for key, value in batch_items:
        # Update the exact frequencies of all distinct items
        if key not in histogram:
            histogram[key] = value
        else:
            histogram[key] = histogram[key] + value

        # &&&&&&&&&&&&&&&&&&&&&&&&
        # COUNT SKETCH ALGORITHM
        # &&&&&&&&&&&&&&&&&&&&&&&&

        # Update the array
        for j in range(D):
            # Get the value of hash function h for item 'key' and row j
            a = hash_parameters_h[j][0]
            b = hash_parameters_h[j][1]
            h = hc(a, b, p, W, key)
            # Get the value of hash function g for item 'key' and row j
            a = hash_parameters_g[j][0]
            b = hash_parameters_g[j][1]
            ris = hc(a, b, p, 2, key)
            g = -1
            # If the value returned by the hash function g is even assign g = 1
            if(ris % 2 == 0):
                g = 1
            C[j][h] = C[j][h] + g*value


    # Check the total stream length
    if totalStreamLength[0] >= THRESHOLD:
        stopping_condition.set()




if __name__ == '__main__':
    # Check command line arguments
    assert len(sys.argv) == 7, "USAGE: python3 G014HW3.py <D> <W> <left> <right> <K> <portExp>"

    conf = SparkConf().setMaster("local[*]").setAppName("G014HW3")

    sc = SparkContext(conf=conf)
    ssc = StreamingContext(sc, 1)  # Batch duration of 1 second
    ssc.sparkContext.setLogLevel("ERROR")

    stopping_condition = threading.Event()


    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    # INPUT READING
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    D = sys.argv[1]
    assert D.isdigit(), "D must be an integer"
    D = int(D)
    W = sys.argv[2]
    assert W.isdigit(), "W must be an integer"
    W = int(W)
    left = sys.argv[3]
    assert left.isdigit(), "left must be an integer"
    left = int(left)
    right = sys.argv[4]
    assert right.isdigit(), "right must be an integer"
    right = int(right)
    K = sys.argv[5]
    assert K.isdigit(), "K must be an integer"
    K = int(K)
    portExp = sys.argv[6]
    assert portExp.isdigit(), "portExp must be an integer"
    portExp = int(portExp)


    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    # DEFINING THE REQUIRED DATA STRUCTURES TO MAINTAIN THE STATE OF THE STREAM
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    R = [left,right] # Range of accepted values

    streamLengthFiltered = [0] # Filtered stream length
    totalStreamLength = [0] # Total stream length
    histogram = {} # Hash Table for the exact frequencies
    hash_parameters_h = {} # Hash Table for the parameters of the D hash functions h
    hash_parameters_g = {} # Hash Table for the parameters of the D hash functions g

    # Count sketch array initialized with all 0
    C = [[0 for i in range(W)] for j in range(D)]

    # Definition of 2*D hash function
    p = 8191
    for i in range(D):
        a = rand.randint(1,p-1)
        b = rand.randint(0,p-1)
        hash_parameters_h[i] = [a,b]
        a = rand.randint(1,p-1)
        b = rand.randint(0,p-1)
        hash_parameters_g[i] = [a,b]


    # CODE TO PROCESS AN UNBOUNDED STREAM OF DATA IN BATCHES
    stream = ssc.socketTextStream("algo.dei.unipd.it", portExp, StorageLevel.MEMORY_AND_DISK)

    stream.foreachRDD(lambda batch: process_batch(batch))

    # MANAGING STREAMING SPARK CONTEXT
    ssc.start()

    stopping_condition.wait()

    ssc.stop(False, True)


    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    # COMPUTE FINAL STATISTICS
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    # Compute the true and approximate second moment (normalized)
    trueSecondMoment = 0
    apprSecondMoment = 0

    # True F2
    for u in histogram:
        trueSecondMoment += histogram[u]**2

    # Approximate F2
    F2j = []
    for j in range(D):
        value = 0
        for k in range(W):
            value += C[j][k]**2
        F2j.append(value)

    apprSecondMoment = stat.median(F2j)

    # Normalization
    trueSecondMoment = trueSecondMoment/(streamLengthFiltered[0]**2)
    apprSecondMoment = apprSecondMoment/(streamLengthFiltered[0]**2)


    # Retrieve the most frequent K items and the corresponding values
    more_frequent = get_first_k(histogram, K)


    # Calculate the average relative error
    average_relative_error = 0
    appr_frequencies = {}

    for u in more_frequent:
        # Retrieve the approximate frequency of item u
        values = []
        for j in range(D):
            # Get the value of hash function h for item 'u' and row j
            a = hash_parameters_h[j][0]
            b = hash_parameters_h[j][1]
            h = hc(a,b,p,W,u)
            # Get the value of hash function g for item 'u' and row j
            a = hash_parameters_g[j][0]
            b = hash_parameters_g[j][1]
            ris = hc(a,b,p,2,u)
            g = -1
            # If the value returned by the hash function g is even assign g = 1
            if(ris % 2 == 0):
                g = 1
            value = C[j][h]*g
            values.append(value)

        appr_frequencies[u] = stat.median(values)

        # Calculate the relative error
        relative_error = abs(more_frequent[u]-appr_frequencies[u])/more_frequent[u]
        average_relative_error += relative_error

    average_relative_error = average_relative_error/K


    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    # PRINT
    # &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    print("D =", D, "W =", W, "[left,right] =", R, "K =", K, "Port =", portExp)

    print("Total number of items =", totalStreamLength[0])
    print("Total number of items in =", R, "=", streamLengthFiltered[0])
    print("Number of distinct items in", R, "=", len(histogram))

    if K <= 20:
        for u in more_frequent:
            print("Item", u, "Freq =", more_frequent[u], "Est. Freq =", appr_frequencies[u])

    print("Avg err for top", K, "=", average_relative_error)
    print("F2 =", trueSecondMoment, "F2 Estimate =", apprSecondMoment)
