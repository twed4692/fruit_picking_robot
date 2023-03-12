/*--------------------------------------------------------------------------------------------------------
An adjacency matrix that defines the orchard and is within the movement controller.

The adjacency matrix represents a node graph with weighted edges and which also indicates the angle between
nodes in terms of the global reference frame. It is capable of generating nodes and edge weightings as well
as the angular direction in the global reference frame between nodes. It is also able to return the weightings
and directions between nodes and the total number of vertices/nodes in the node graph.

Initial revision: 500488214
--------------------------------------------------------------------------------------------------------*/


#ifndef ADJACENCYMATRIX_H
#define ADJACENCYMATRIX_H

#include <utility>              // Container needed to make and have pairs (for the adjacency matrix)
#include <limits.h>             // Needed for max integer for Dijkstra's algorithm comparison of distances

class CAdjacencyMatrix 
{

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        // Constructor for the adjacency matrix
        CAdjacencyMatrix();

        // Get the edge weighting between a source and destination node
        double GetWeighting(int source, int destination);

        // Function to get the total number of vertices/nodes
        int GetVertices();

        // Get the angular direction between a source and destination node
        int GetDirection(int source, int destination);

        // Enums for directions
        enum eDirections                        // Different possible angular directions between nodes
        {
            FORWARDS = 0,
            RIGHT = 90,
            BACKWARDS = 180,
            LEFT = 270                        
        };

    private:
        /*------------------------------------------------------------------------------------------------
        Internal variables
        ------------------------------------------------------------------------------------------------*/
        // The number of vertices/nodes in the adjacency matrix
        static constexpr int _vertices = 9;

        // The short and long edges between nodes
        enum eWeightings
        {
            SHORT = 1,
            LONG = 2
        };

        // Make a zero pair where nodes cannot be travelled btween 
        const std::pair<int, int> _ZERO = std::make_pair(0, 0);

        // Define the adjacency matrix of pairs with the first number corresponding to the weighting
        // between edges and the second number corresponding to the angle between nodes. The rows of 
        // the matrix are the source nodes and the columns are the destination nodes
        std::pair<int, int> _matrix[_vertices][_vertices];
};

#endif