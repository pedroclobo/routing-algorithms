/******************************************************************************\
* Distance vector routing protocol without reverse path poisoning.             *
\******************************************************************************/

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "routing-simulator.h"

// Message format to send between nodes.
typedef struct {
	cost_t dv[MAX_NODES];
} data_t;

// State format.
typedef struct {
	cost_t dvs[MAX_NODES][MAX_NODES];
	node_t via[MAX_NODES];
} state_t;

// Recompute distance vector.
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
			if (COST_ADD(get_link_cost(z), state->dvs[z][y]) < min_cost) {
				min_cost = COST_ADD(get_link_cost(z), state->dvs[z][y]);
				via = z;
			}
		}

		// If min_cost is different from the distance vector value, update it.
		// If via is different from the previous via, update it, but signal no changes in the distance vector.
		bool changed_dv = min_cost != state->dvs[get_current_node()][y];
		bool changed_via = state->dvs[get_current_node()][y] != COST_INFINITY && state->via[y] != via;
		if (changed_dv || changed_via) {
			// Distance vector changed.
			if (changed_dv) {
				changed = true;
			}

			// Update distance vector and via, and set route.
			state->dvs[get_current_node()][y] = min_cost;
			if (min_cost != COST_INFINITY) {
				state->via[y] = via;
			} else {
				state->via[y] = -1;
			}
			set_route(y, via, min_cost);
		}
	}

	return changed;
}

// Send message to neighbors.
void send_messages() {
	state_t *state = (state_t *)get_state();

	// Create message.
	message_t message;
	message.data = (data_t *)malloc(sizeof(data_t));
	memcpy(message.data, state->dvs[get_current_node()], sizeof(data_t));
	message.size = sizeof(message.data);

	for (node_t neighbor = get_first_node(); neighbor <= get_last_node(); neighbor = get_next_node(neighbor)) {
		if (get_link_cost(neighbor) < COST_INFINITY && neighbor != get_current_node()) {
			send_message(neighbor, message);
		}
	}
}

// Handler for the node to allocate and initialize its state.
void *init_state() {
	state_t *state = (state_t *)malloc(sizeof(state_t));

	// Initialize distance vector.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		state->dvs[get_current_node()][node] = get_link_cost(node);
	}

	// Initialize the distance vector of the other nodes.
	for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
		for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
			if (node1 == get_current_node()) {
				break;
			}
			state->dvs[node1][node2] = COST_INFINITY;
		}
	}

	return state;
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
	// Recompute distance vector.
	bool changed = bellman_ford();

	// Send message to neighbors if distance vector changed.
	if (changed) {
		send_messages();
	}
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
	state_t *state = (state_t *)get_state();

	// Copy new distance vector from message to state.
	data_t *data = (data_t *)message.data;
	memcpy(state->dvs[sender], data->dv, sizeof(data->dv));

	// Recompute distance vector.
	bool changed = bellman_ford();

	// Send message to neighbors if distance vector changed.
	if (changed) {
		send_messages();
	}
}
