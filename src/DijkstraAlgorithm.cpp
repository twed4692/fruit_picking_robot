/*--------------------------------------------------------------------------------------------------------
C++ implementation of DijkstraAlgorithm class. This function is capable of using a known weighted node 
graph to find the shortest route from a source node to a destination node. 

- The GetPath function finds all possible paths from a source to destination node and then will iterate 
through these possible routes to find the shortest path. 
- The PrintPath vector will create the shortest path vector. This vector will then be outputted to other 
class when the public function GetPath is called.

Initial revision: 500488214

Code intially from https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
Changes were made so that the shortest path to a destination rather than all other nodes would be printed.
--------------------------------------------------------------------------------------------------------*/

#include "DijkstraAlgorithm.h"

/*--------------------------------------------------------------------------------------------------------
Constructor: When the constructor is called the known node graph will be saved as a member variable to
the Dijkstra's algorithm class. The total number of nodes in the known node graph will also be saved as a 
member variable.
--------------------------------------------------------------------------------------------------------*/
CDijkstraAlgorithm::CDijkstraAlgorithm(CAdjacencyMatrix* nodeGraph) 
{

    _nodeGraph = nodeGraph;
    _vertices = _nodeGraph->GetVertices();
}


/*--------------------------------------------------------------------------------------------------------
Save the shortest route in terms of nodes to a vector.
--------------------------------------------------------------------------------------------------------*/
void CDijkstraAlgorithm::PrintPath(int currentVertex, std::vector<int> parents)
{

	// Set the current vertex as the base node as it will have no parent
	if (currentVertex == _NO_PARENT) 
	{
		return;
	}

	// Recursively call this function to get the next node of the shortest route
	PrintPath(parents[currentVertex], parents);

	// Push back the next node in the shortest route to the shortest route vector
    _shortestPath.push_back(currentVertex);
}


/*--------------------------------------------------------------------------------------------------------
This function will implement Dijkstra's algorithm and find the shortest path from a source node to a 
destination node. It will start by looping through and finding all possible routes between the two nodes 
and the corresponding distance associated with a given route. It will then loop through all of these to 
find the shortest route and save the order of nodes for this route.
--------------------------------------------------------------------------------------------------------*/
std::vector<int> CDijkstraAlgorithm::GetPath(int startVertex, int destination)
{

	/* Clear the current shortest path vector so that a new route can be saved between a 
	different start and destination node */
    _shortestPath.clear();

	// Vector to hold the shortest distance from the source node i in shortestDistances[i]
	std::vector<float> shortestDistances(_vertices);

	/* added[i] will be true if the vertex i is included in the shortest path tree or the shortest
	distance from the source to node i is finalised. */ 
	std::vector<bool> added(_vertices);

	// Initialise all distances from source node to node i as infinity and added[] as false
	for (int vertexIndex = 0; vertexIndex < _vertices; vertexIndex++) 
	{
		shortestDistances[vertexIndex] = INT_MAX;
		added[vertexIndex] = false;
	}

	// The distance from the source to itself is always zero
	shortestDistances[startVertex] = 0;

	// A parent array to store the shortest path tree
	std::vector<int> parents(_vertices);

	// The source vertex will not have a parent
	parents[startVertex] = _NO_PARENT;

	// Find the shortest path for all vertices
	for (int i = 1; i < _vertices; i++) 
	{
		// Pick the closes vertex from the set of vertices not yet processed. The nearest vertex in the 
		// first iteration will always be the source node.
		int nearestVertex = _NO_PARENT;

		// The shortest distance will start of as infinity
		int shortestDistance = INT_MAX;

		// Looop through all vertices
		for (int vertexIndex = 0; vertexIndex < _vertices; vertexIndex++) 
		{

			// If a vertex is not yet processed and is the closest it will be saved as the closest vertex
			if (!added[vertexIndex] && shortestDistances[vertexIndex] < shortestDistance) 
			{

				// Set the nearest vertex as the current vertex
				nearestVertex = vertexIndex;

				// Set the new shortest distance as the distance between the source and current vertex
				shortestDistance = shortestDistances[vertexIndex];
			}
		}

		// Set the selected (closest) vertex as processed
		added[nearestVertex] = true;

		// Update the distance value of the adjacent vertices of the picked vertex. 
		for (int vertexIndex = 0; vertexIndex < _vertices; vertexIndex++) 
		{

			// Get the distance between the two desired nodes
			int edgeDistance = _nodeGraph->GetWeighting(nearestVertex, vertexIndex);

			// Check that the edge distance is not zero (the nodes are adjacent/connected) and the sum of 
			// the shortest distance and the current distance between two nodes is less than the currently 
			// saved shortest distance.
			if (edgeDistance > 0 && ((shortestDistance + edgeDistance) < shortestDistances[vertexIndex])) 
			{
				// If this is the new shortest distance save this nearest vertex as the parent
                parents[vertexIndex] = nearestVertex;

				// Save the current cumulative shortest distance
				shortestDistances[vertexIndex] = shortestDistance + edgeDistance;

			}
		}
	}

	// printSolution(startVertex, destination, shortestDistances, parents);
    PrintPath(destination, parents);

	// Return the shortest path vector to other classes
    return _shortestPath;
}