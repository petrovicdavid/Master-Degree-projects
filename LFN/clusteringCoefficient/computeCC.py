# Andrea Ghiotto 2118418, David Petrovic 2092073
# Learning From Network project
# Implementation of the computation of the clustering coefficients

import sys
import scipy.special
import os

def get_file(filename):
    substring1 = "locality"
    substring2 = "T-Sample"
    substring3 = "eigen"
    filename = filename.replace(".txt", "")
    if substring1 in filename:
        filename = filename.replace("../locality-aware/", "")
        filename = filename + "_locality.txt"
    if substring2 in filename:
        filename = filename.replace("../T-Sample/", "")
        filename = filename + "_sample.txt"
    if substring3 in filename:
        filename = filename.replace("../eigenTriangle/", "")
        filename = filename + "_eigen.txt"
    return filename

if __name__ == "__main__":
    # Check the number of input parameters.
    assert len(sys.argv) == 2, "Usage: python computeCC.py <file_name>"

    # Read the file_name.
    filepath = sys.argv[1]
    assert os.path.isfile(filepath), "File not found!"

    separator = " "
    results = []
    
    with open(filepath, 'r') as file:
        results = [tuple(map(int, line.strip().split(separator)[:2])) for line in file]

    clustering_coefficients = []

    for tupla in results:
        edges, triangles = tupla
        
        cc = triangles / (6*scipy.special.binom(edges, 3))
        clustering_coefficients.append((edges, cc))

        print("Clustering coefficient: " + str(cc))

    # Save the results (number of edges and approximate number of triangles) in the file.
    result_file = get_file(filepath)
    with open(result_file, "a") as file:
        for tupla in clustering_coefficients:
            file.write(str(tupla[0]) + " " + str(tupla[1]) + "\n")

    print("Results saved!")