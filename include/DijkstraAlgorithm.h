/*--------------------------------------------------------------------------------------------------------
Dijkstra's algorithm class will know a weighted node graph and from this information be able to output
a vector of ints which represent the shortest route between nodes in terms of which nodes to traverse.

The algorithm will find the shortest route by taking a source node and finding the shortest distance to all 
adjacent node, then looping through all adjacent nodes of those nodes and storing the shortest distance from 
the source node into an array.

Initial revision: 500488214
--------------------------------------------------------------------------------------------------------*/

#ifndef DIJKSTRAALGORITHM_H
#define DIJKSTRAALGORITHM_H

#include "AdjacencyMatrix.h"                 
#include <vector>               // Vector container needed to store vector of ints representign shortest path
#include <limits.h>             // Included for max integer value for Dijkstra's algorithm (for comparisons)

class CDijkstraAlgorithm 
{

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        // Constructor to initialise a known node graph 
        CDijkstraAlgorithm(CAdjacencyMatrix* nodeGraph);  

        // Returns shortest route as a vector of nodes (ints)          
        std::vector<int> GetPath(int startVertex, int destination); 

    private:
        /*------------------------------------------------------------------------------------------------
        Internal functions
        ------------------------------------------------------------------------------------------------*/
        // Function to create the shortest route vector
        void PrintPath(int currentVertex, std::vector<int> parents);   
        
        /*------------------------------------------------------------------------------------------------
        Internal variables
        ------------------------------------------------------------------------------------------------*/
        int _vertices;                              // The total number of nodes in the known node graph
        const int _NO_PARENT = -1;                  // Integer to indicate a node is the source node
        std::vector<int> _shortestPath;             // Vector saving the shortest path
        CAdjacencyMatrix* _nodeGraph;               // Pointer to the known node graph (adjacency matrix)
};

#endif
