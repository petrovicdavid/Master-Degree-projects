# Andrea Ghiotto 2118418, David Petrovic 2092073
# Learning From Network project
# Implementation of eigenTriangle algorithm

import numpy as np 
import networkx as nx
import sys
import os
from scipy.sparse.linalg import eigsh

def create_adjacency_matrix_sparse(edge_list):
    G = nx.Graph()
    G.add_edges_from(edge_list)
    A = nx.to_scipy_sparse_array(G)
    A = A.asfptype()
    return A

def condition(eigenvalues, i):
   tol = 0.05
   num = abs(eigenvalues[i-1]**3)
   den = sum(x**3 for x in eigenvalues[:i])
   value = num/den

   if value >= 0 and value <= tol:
      return False
   else:
      return True
   
def file_delimitator(filename):
    with open(filename, 'r') as file:
        first_rows = [file.readline() for _ in range(2)]  # Read the first two rows

    # Verify if there is at least one tab in the first two rows.
    tab = any('\t' in row for row in first_rows)

    # Return the correct delimatator.
    return '\t' if tab else ' '

def get_dataset(filename):
    filename = filename.replace("../dataset/", "")
    return filename

if __name__ == "__main__":
    # Check the number of input parameters.
    assert len(sys.argv) == 2, "Usage: python eigenTriangle.py <file_name>"

    # Read the file_name.
    filepath = sys.argv[1]
    assert os.path.isfile(filepath), "File not found!"

    # Check the type of delimitator of the dataset.
    separator = file_delimitator(filepath)
    edges_file = []
    
    # Read the dataset and save the edges into a list.
    with open(filepath, 'r') as file:
        edges_file = [tuple(map(int, line.strip().split(separator)[:2])) for line in file]

    edges = len(edges_file)
    print("Number of edges: " + str(edges))

    A = create_adjacency_matrix_sparse(edges_file)

    # This function is a wrapper to the ARPACK SSEUPD and DSEUPD functions which use
    # the Implicitly Restarted Lanczos Method to find the eigenvalues and eigenvectors.
    # It finds k eigenvalues and eigenvectors of the real symmetric square matrix.
    eigenvalues, vecs = eigsh(A, k = 50)

    # For searching the top k eigenvalues they must be processed in decrescent order, considering 
    # their absolute value but, after that, they must be processed as they are (not absolute values).
    eigenvalues_sorted = sorted(eigenvalues, key=abs, reverse=True)

    n = len(eigenvalues_sorted)
    i = 3

    # Eigenvalues selection.
    while(condition(eigenvalues_sorted, i) and i < n):
        i += 1

    triangles = int(sum(x**3 for x in eigenvalues_sorted[:i])/6)

    print("Number of eigenvalues used: " + str(i))
    print("Approximate number of triangles: " + str(triangles))

    # Save the results (number of edges and approximate number of triangles) in the result file.
    result_file = "result_" + get_dataset(filepath)
    with open(result_file, "a") as file:
        file.write(str(edges) + " " + str(triangles) + "\n")

    print("Results saved!")