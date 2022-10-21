#include "route_planner.h"
#include <algorithm>


/**
 * @brief Construct a RoutePlanner
 *
 * Find and save nearest start and end nodes based on their x,y coordinates (0 to 100).
 */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the start and end nodes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

/**
 * @brief Compute the heuristic value
 * @param node The node for which we want to compute the heuristic value
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


/**
 * @brief Compute neighbors of current_node and put them in the open_list.
 *
 * After computing the neighbors of current_node, for each of them
 * set current_node as parent, compute heuristic and cost values,
 * mark as visited and push back in the open_list.
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* n : current_node->neighbors) {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value = current_node->g_value + current_node->distance(*n);
        n->visited = true;
        open_list.emplace_back(n);
    }
}


/**
 * @brief Sort open_list on descending f value (g+h) and return the node with lowest f value.
 */
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* lhs, const RouteModel::Node* rhs) {
        return (lhs->g_value + lhs->h_value) >= (rhs->g_value + rhs->h_value);
    });
    RouteModel::Node *lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}


/**
 * @brief Return the path from starting node to current_node
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // While we reach the root, increase distance and push node into path
    while (current_node != nullptr) {
        distance += current_node->parent == nullptr ? 0 : current_node->distance(*current_node->parent);
        path_found.emplace_back(*current_node);
        current_node = current_node->parent;
    };

    // Since we want the path to begin with starting node, we need to reverse it
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


/**
 * @brief A* search algorithm.
 */
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Push the starting node in the open_list
    start_node->visited = true;
    open_list.emplace_back(start_node);

    // While the open_list is non-empty
    while (!open_list.empty()) {
        // get current_node from open_list using NextNode
        current_node = NextNode();

        // add neighbors to open_list
        AddNeighbors(current_node);

        // if we reached the end node, fine
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);

            return;
        }
    }
}
