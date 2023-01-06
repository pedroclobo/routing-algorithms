/******************************************************************************\
* Path vector routing protocol.                                                *
\******************************************************************************/

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "routing-simulator.h"

// Each entry has a cost to get to the node, a path and a path length.
typedef struct {
	cost_t cost;
	node_t *path;
	size_t length;
} entry_t;

// Message format to send between nodes.
typedef struct {
	entry_t *entry;
} data_t;

// State format.
typedef struct {
	entry_t **entries;
} state_t;

// Detect loops in the path.
bool is_loop(node_t via, node_t to) {
	state_t *state = (state_t *)get_state();

	size_t path_size = state->entries[via][to].length;

	for (size_t i = 0; i < path_size - 1; i++) {
		if (state->entries[via][to].path[i] == get_current_node()) {
			return true;
		}
	}

	return false;
}

// Recompute path vector.
bool bellman_ford() {
	state_t *state = (state_t *)get_state();

	bool changed = false;
	cost_t min_cost;
	node_t via;

	// D_x(y) = min { D_x(y), c(x,z) + D_z(y) }
	for (node_t y = get_first_node(); y <= get_last_node(); y = get_next_node(y)) {
		if (y == get_current_node()) {
			continue;
		}

		min_cost = get_link_cost(y);
		via = y;

		// Find the minimum cost to reach y, and the neighbor that allows it.
		for (node_t z = get_first_node(); z <= get_last_node(); z = get_next_node(z)) {
			if (z == get_current_node() || z == y) {
				continue;
			}
			if (COST_ADD(get_link_cost(z), state->entries[z][y].cost) < min_cost && !is_loop(z, y)) {
				min_cost = COST_ADD(get_link_cost(z), state->entries[z][y].cost);
				via = z;
			}
		}

		// If min_cost is different from the path vector value, update it.
		// If via is different from the previous via, update it, but signal no changes in the path vector.
		bool changed_dv = min_cost != state->entries[get_current_node()][y].cost;
		bool changed_via = state->entries[get_current_node()][y].path[0] != via && state->entries[get_current_node()][y].cost != COST_INFINITY;
		if (changed_dv || changed_via) {
			changed = true;

			// Update cost.
			state->entries[get_current_node()][y].cost = min_cost;
			set_route(y, via, min_cost);

			// If cost is COST_INFINITY, there's no path.
			if (min_cost == COST_INFINITY) {
				state->entries[get_current_node()][y].length = 0;
				continue;
			}

			// Copy path and set path length.
			// Path starts with via.
			size_t path_size = 0;
			state->entries[get_current_node()][y].path[path_size++] = via;
			if (via != y) {
				for (size_t i = 0; i < state->entries[via][y].length; i++) {
					state->entries[get_current_node()][y].path[path_size++] = state->entries[via][y].path[i];
				}
			}
			state->entries[get_current_node()][y].length = path_size;
		}
	}

	return changed;
}

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
		message.size = sizeof(message.data);

		// Allocate memory.
		data_t *data = (data_t *)message.data;
		data->entry = (entry_t *)malloc(sizeof(entry_t) * MAX_NODES);
		for (node_t node = 0; node <= get_last_node(); node = get_next_node(node)) {
			data->entry[node].path = (node_t *)malloc(sizeof(node_t) * MAX_NODES);
		}

		// Copy entries.
		memcpy(data->entry, state->entries[get_current_node()], sizeof(entry_t) * MAX_NODES);

		send_message(neighbor, message);
	}
}

// Handler for the node to allocate and initialize its state.
void *init_state() {
	state_t *state = (state_t *)malloc(sizeof(state_t));

	// Allocate memory.
	state->entries = (entry_t **)malloc(sizeof(entry_t *) * MAX_NODES);
	for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
		state->entries[node1] = (entry_t *)malloc(sizeof(entry_t) * MAX_NODES);
		for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
			state->entries[node1][node2].path = (node_t *)malloc(sizeof(node_t) * MAX_NODES);
		}
	}

	// Initialize path vector.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		// Set cost.
		state->entries[get_current_node()][node].cost = get_link_cost(node);

		// Current node is the only one in the path.
		if (node == get_current_node()) {
			state->entries[get_current_node()][node].length = 0;
			continue;
		}

		if (get_link_cost(node) == COST_INFINITY) {
			// Not neighbor, so the path is empty.
			state->entries[get_current_node()][node].length = 0;

		} else {
			// Path to neighbor is myself + neighbor
			state->entries[get_current_node()][node].path[0] = node;
			state->entries[get_current_node()][node].length = 1;
		}
	}

	// Initialize the path vector of the other nodes.
	// Each one gets a cost of COST_INFINITY and a path length of 0.
	for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
		if (node1 == get_current_node()) {
			continue;
		}
		for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
			state->entries[node1][node2].cost = COST_INFINITY;
			state->entries[node1][node2].length = 0;
		}
	}

	return state;
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
	// Recompute path vector.
	bool changed = bellman_ford();

	// Send message to neighbors if path vector changed.
	if (changed) {
		send_messages();
	}
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
	state_t *state = (state_t *)get_state();

	// Copy new path vector from message to state.
	data_t *data = (data_t *)message.data;
	memcpy(state->entries[sender], data->entry, sizeof(entry_t) * MAX_NODES);

	// Recompute path vector.
	bool changed = bellman_ford();

	// Send message to neighbors if path vector changed.
	if (changed) {
		send_messages();
	}
}
