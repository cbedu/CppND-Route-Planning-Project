#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto & temp_node : current_node->neighbors)
    {
        temp_node->h_value = CalculateHValue(temp_node);
        temp_node->parent = current_node;
        temp_node->g_value = current_node->g_value + temp_node->distance(*current_node); //existing gVal + intermediate distance
        open_list.emplace_back(temp_node); //same technique used in FindNeighbors to populate.
        temp_node->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

//// My helper notes
// - Make of use of sort(v->begin(), v->end(), Compare) logic...
// - Compare:  bool Compare(const vector<int> a, const vector<int> b) { comparison logic; return true/false } 

// Sort by lower
bool CompareNodeDistance(const RouteModel::Node* a, const RouteModel::Node* b) {
    float a_dist = a->g_value + a->h_value;
    float b_dist = b->g_value + b->h_value;
    return a_dist > b_dist;
}

RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node* next_node = NULL;
    float next_node_total_dist;

    std::sort(open_list.begin(), open_list.end(), CompareNodeDistance);

    next_node = open_list.back();
    open_list.pop_back();

    #ifdef COUT_NEXTNODE_DEBUG
    //std::cout << "Next node distance: h = " << next_node->h_value << "  g = " << next_node->g_value;
    std::cout << "Co-ords: x<" << next_node->x << "> y<" << next_node->y << ">" << std::endl;
    #endif

    return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    
    // path_found will be populated by parents until complete chain
    // grab current node (which is the goal node)
    RouteModel::Node* active_node = current_node;

    // (while active node is not start node, add to back of list, add distance from parent to distance val, then grab parent and loop)
    while(active_node != start_node)
    {
        path_found.push_back(*active_node);
        distance += active_node->distance(*(active_node->parent));
        active_node = active_node->parent;
    }
    // Path is no populated with first node being end destination, and last node being start. Need to reverse as per TODO 6
    std::reverse(path_found.begin(), path_found.end()); // https://www.geeksforgeeks.org/how-to-reverse-a-vector-using-stl-in-c/

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // set up the initial state,
    // we are on start_node,
    // need to add as first entry on list.
    // then while we aren't on end node need to add neighbors and run NextNode to get next favourite nearest
    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <1>";
    #endif
    open_list.push_back(start_node);
    current_node = start_node;

    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <2>";
    #endif
    while(current_node != end_node)
    {
    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <3>";
    #endif
        AddNeighbors(current_node);
    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <4>";
    #endif
        current_node = NextNode();
    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <5>";
    #endif
    }

    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <6>";
    #endif
    // We should have a path now.
    m_Model.path = ConstructFinalPath(current_node);

    #ifdef COUT_ASTAR_DEBUG
    std::cout << "Got to <7>";
    #endif
}