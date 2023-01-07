/******************************************************************************\
* Link state routing protocol.                                                 *
\******************************************************************************/

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "routing-simulator.h"

typedef struct {
	cost_t link_cost[MAX_NODES];
	int version;
} link_state_t;

// Message format to send between nodes.
typedef struct {
	link_state_t ls[MAX_NODES];
} data_t;

// State format.
typedef struct {
	cost_t cost[MAX_NODES][MAX_NODES];
	node_t via[MAX_NODES];
	int version[MAX_NODES];
} state_t;

// Send message to neighbors.
void send_messages() {
	state_t *state = (state_t *)get_state();

	for (node_t neighbor = get_first_node(); neighbor <= get_last_node(); neighbor = get_next_node(neighbor)) {
		// Send message only to neighbors.
		if (neighbor == get_current_node() || get_link_cost(neighbor) == COST_INFINITY) {
			continue;
		}

		// Create message.
		message_t message;
		message.data = (data_t *)malloc(sizeof(data_t));
		message.size = sizeof(data_t);
		data_t *data = (data_t *)message.data;

		// Copy link state.
		for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
			data->ls[node1].version = state->version[node1];
			for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
				data->ls[node1].link_cost[node2] = state->cost[node1][node2];
			}
		}

		send_message(neighbor, message);
	}
}

// Check if node is in the tree.
bool is_in_tree(node_t node, node_t *tree, size_t size) {
	for (size_t i = 0; i < size; i++) {
		if (tree[i] == node) {
			return true;
		}
	}

	return false;
}

// Find the minimum cost node that is not in the tree.
// Set min_node and min_cost to the corresponding values.
void min(cost_t matrix[][MAX_NODES], node_t *tree, size_t size, node_t *min_node, cost_t *min_cost) {
	*min_cost = COST_INFINITY;
	*min_node = -1;

	// Iterate over all nodes.
	for (node_t node = get_first_node(); node <= get_last_node(); node++) {
		if (is_in_tree(node, tree, size)) {
			continue;
		}

		if (matrix[get_current_node()][node] >= *min_cost && *min_node != -1) {
			continue;
		}

		// Found new minimum.
		*min_cost = matrix[get_current_node()][node];
		*min_node = node;
	}
}

// Get the next hop for a destination.
node_t get_via(cost_t matrix[][MAX_NODES], node_t *predecessors, node_t destination) {
	// The node can't be reached.
	if (matrix[get_current_node()][destination] == COST_INFINITY) {
		return -1;
	}

	node_t aux = destination;
	while (predecessors[aux] != get_current_node()) {
		aux = predecessors[aux];
	}

	return aux;
}

void dijkstra() {
	state_t *state = (state_t *)get_state();

	// Initialize predecessors.
	node_t predecessors[MAX_NODES];
	for (node_t node = 0; node <= get_last_node(); node = get_next_node(node)) {
		predecessors[node] = get_current_node();
	}

	// Initialize cost matrix.
	cost_t matrix[MAX_NODES][MAX_NODES];
	for (node_t node1 = 0; node1 <= get_last_node(); node1 = get_next_node(node1)) {
		for (node_t node2 = 0; node2 <= get_last_node(); node2 = get_next_node(node2)) {
			matrix[node1][node2] = state->cost[node1][node2];
		}
	}

	// Start by checking the current node.
	int size = 0;
	node_t tree[MAX_NODES];
	tree[size++] = get_current_node();

	// While all the nodes are not in the tree.
	while (size != get_last_node() + 1) {
		cost_t min_cost = COST_INFINITY;
		node_t w = -1;

		// Find the node with the minimum cost that is not in the tree.
		min(matrix, tree, size, &w, &min_cost);

		// Add it to the tree.
		tree[size++] = w;

		// Iterate over every node x not in the tree and update the cost.
		// D[x] = min{ D[x], (D[w] + c[w][x]) }
		for (node_t x = get_first_node(); x <= get_last_node(); x = get_next_node(x)) {
			if (is_in_tree(x, tree, size)) {
				continue;
			}

			// Update cost.
			cost_t new_cost = COST_ADD(matrix[get_current_node()][w], matrix[w][x]);
			if (new_cost < matrix[get_current_node()][x]) {
				matrix[get_current_node()][x] = new_cost;
				predecessors[x] = w;
			}
		}
	}

	// Update nodes.
	for (int n = 0; n < size; n++) {
		node_t node = tree[n];
		node_t via = get_via(matrix, predecessors, node);

		// Already up to date.
		if (node == get_current_node() || (state->cost[get_current_node()][node] == matrix[get_current_node()][node] && state->via[node] == via)) {
			continue;
		}

		// Update via and set route.
		state->via[node] = via;
		set_route(node, via, matrix[get_current_node()][node]);
	}
}

// Handler for the node to allocate and initialize its state.
void *init_state() {
	state_t *state = (state_t *)calloc(1, sizeof(state_t));

	// Initialize versions.
	// Current node gets version 1, all other nodes get version 0.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		if (node == get_current_node()) {
			state->version[node] = 1;
		} else {
			state->version[node] = 0;
		}
	}

	// Initialize all hops to -1.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		state->via[node] = -1;
	}

	// Initialize costs for current node.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		state->cost[get_current_node()][node] = get_link_cost(node);
	}

	// Initialize costs for all other nodes.
	for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
		if (node1 == get_current_node()) {
			continue;
		}
		for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
			state->cost[node1][node2] = COST_INFINITY;
		}
	}

	return state;
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
	state_t *state = (state_t *)get_state();

	// Update cost and increment version.
	state->cost[get_current_node()][neighbor] = new_cost;
	state->version[get_current_node()]++;

	// Recompute routes and send message to neighbors.
	dijkstra();
	send_messages();
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
	state_t *state = (state_t *)get_state();
	data_t *data = (data_t *)message.data;

	bool changed = false;

	for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
		// Don't recompute routes as the version is outdated.
		if (data->ls[node1].version <= state->version[node1]) {
			continue;
		}

		// Copy new version of routes.
		state->version[node1] = data->ls[node1].version;
		for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
			state->cost[node1][node2] = data->ls[node1].link_cost[node2];
		}

		changed = true;
	}

	// Recompute routes and send message to neighbors.
	if (changed) {
		dijkstra();
		send_messages();
	}
}
