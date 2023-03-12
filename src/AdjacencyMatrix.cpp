/*--------------------------------------------------------------------------------------------------------
C++ implementation of AdjacencyMatrix class.

It is capable of generating the desired nodes and the weighted edges and angular directions between nodes
in terms of a general reference frame.

This class is also capable of returning the weightings between a source and destination node and the angle
between a source and reference node. This returned value can then be used by Dijkstra's algorithm to 
determine a shortest path.

Initial revision: 500488214
--------------------------------------------------------------------------------------------------------*/

#include "AdjacencyMatrix.h"

/*--------------------------------------------------------------------------------------------------------
Constructor: When the constructor is called a node graph with 9 nodes and predetermined edges between the 
nodes is created. The edge weightings and the angular direction between nodes is predetermined.

Create the predetermined node graph including edge weights and angular directions between nodes. 
This was done by turning the node graph into a weighted adjacency matrix that also included the angular 
direction between nodes. The first element of each pair is the edge weighting and the second element is 
the direction.
--------------------------------------------------------------------------------------------------------*/
CAdjacencyMatrix::CAdjacencyMatrix() : _matrix ({
    {   
        _ZERO, std::make_pair(LONG, FORWARDS), _ZERO, _ZERO,
        _ZERO, std::make_pair(SHORT, RIGHT), _ZERO, _ZERO, _ZERO    
    },

    {   
        std::make_pair(LONG, BACKWARDS), _ZERO, std::make_pair(LONG, FORWARDS), _ZERO,
        std::make_pair(SHORT, RIGHT), _ZERO, _ZERO, _ZERO, _ZERO
    },

    {
        _ZERO, std::make_pair(LONG, BACKWARDS), _ZERO, std::make_pair(SHORT, RIGHT),
        _ZERO, _ZERO, _ZERO, _ZERO, _ZERO
    },

    {
        _ZERO, _ZERO, std::make_pair(SHORT, LEFT), _ZERO,
        std::make_pair(LONG, BACKWARDS), _ZERO, _ZERO, _ZERO, std::make_pair(SHORT, RIGHT)
    },

    {
        _ZERO, std::make_pair(SHORT, LEFT), _ZERO, std::make_pair(LONG, FORWARDS),
        _ZERO, std::make_pair(LONG, BACKWARDS), _ZERO, std::make_pair(SHORT, RIGHT), _ZERO
    },

    {
        std::make_pair(SHORT, LEFT), _ZERO, _ZERO, _ZERO, std::make_pair(LONG, FORWARDS), 
        _ZERO, std::make_pair(SHORT, RIGHT), _ZERO, _ZERO
    },

    {
        _ZERO, _ZERO, _ZERO, _ZERO, _ZERO, std::make_pair(SHORT, LEFT), _ZERO, 
        std::make_pair(LONG, FORWARDS), _ZERO
    },

    {
        _ZERO, _ZERO, _ZERO, _ZERO, std::make_pair(SHORT, LEFT), _ZERO,
        std::make_pair(LONG, BACKWARDS), _ZERO, std::make_pair(LONG, FORWARDS)
    },

    {
        _ZERO, _ZERO, _ZERO, std::make_pair(SHORT, LEFT),
        _ZERO, _ZERO, _ZERO, std::make_pair(LONG, BACKWARDS), _ZERO
    }})
{}

/*--------------------------------------------------------------------------------------------------------
This function returns the edge weighting (distance) between a source and destination node which can then 
be used by other classes to determine the shortest route between nodes.
--------------------------------------------------------------------------------------------------------*/
double CAdjacencyMatrix::GetWeighting(int source, int destination) 
{
    // Return the first element of the pair between two nodes which will be the weighting between the nodes
    return _matrix[source][destination].first;
}

/*--------------------------------------------------------------------------------------------------------
This function returns the angular direction in the global reference frame between a source and destination
node which can then be used by other classes to navigate between nodes.
--------------------------------------------------------------------------------------------------------*/
int CAdjacencyMatrix::GetDirection(int source, int destination) 
{
    // Returns the second element of the pair between nodes which is the angular direction between nodes
    return _matrix[source][destination].second;
}

/*--------------------------------------------------------------------------------------------------------
This function returns the total number of nodes in the adjacency matrix.
--------------------------------------------------------------------------------------------------------*/
int CAdjacencyMatrix::GetVertices() 
{
    // Return the number of vertices
    return _vertices;
}

