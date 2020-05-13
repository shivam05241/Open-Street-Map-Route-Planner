#include "route_planner.h"
#include <algorithm>
RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    RoutePlanner::start_node = &RoutePlanner::m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &RoutePlanner::m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const* node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {

    current_node->FindNeighbors();
    for (auto nod : current_node->neighbors) {
        nod->parent = current_node;
        nod->h_value = CalculateHValue(nd);
        nod->g_value = current_node->distance(*nod) + current_node->g_value;
        open_list.push_back(nod);
        nod->visited = true;
    }
}


// NextNode method to sort the open list and return the next node.
// - Sorting the open_list according to the sum of the h value and g value.
// - Creating a pointer to the node in the list with the lowest sum.
// - Removing that node from the open_list.
// - Return the pointer.
RouteModel::Node* RoutePlanner::NextNode() {
    RouteModel::Node* next;
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* next1, const RouteModel::Node* next2) {
        return (next1->h_value + next1->g_value) > (next2->h_value + next2->g_value);
        });
    next = open_list.back();
    open_list.pop_back();
    return next;
}



// ConstructFinalPath method to return the final path found from your A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node* nod;
    nod = current_node;
    path_found.push_back(*nod);

    // For each node in the path
    while (nod != start_node) {

        // Store node in the path
        path_found.push_back(*(nod->parent));

        // Add the distance
        distance += nod->distance(*(nod->parent));

        // Move to the parent node
        nod = nod->parent;
    }
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}



// Implementing A* Search algorithm here.
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node = nullptr;
    current_node = start_node;
    open_list.push_back(current_node);
    current_node->visited = true;
    while (current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path = ConstructFinalPath(current_node);
}