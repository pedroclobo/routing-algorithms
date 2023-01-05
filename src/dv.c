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

void print_table() {
	state_t *state = (state_t *)get_state();

	fprintf(stderr, "[t=%d]: Table of node %d:\n", get_current_time(), get_current_node());
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		fprintf(stderr, "%d: %d via %d\n", node, state->dvs[get_current_node()][node], state->via[node]);
	}
	fprintf(stderr, "\n");
}

// Handler for the node to allocate and initialize its state.
void *init_state() {
	state_t *state = (state_t *)malloc(sizeof(state_t));

	// Initialize distance vector.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		state->dvs[get_current_node()][node] = get_link_cost(node);
	}

	for (node_t node1 = get_first_node(); node1 <= get_last_node(); node1 = get_next_node(node1)) {
		for (node_t node2 = get_first_node(); node2 <= get_last_node(); node2 = get_next_node(node2)) {
			if (node1 == get_current_node()) {
				break;
			}
			state->dvs[node1][node2] = COST_INFINITY;
		}
	}

	// Create message with distance vector.
	message_t message;
	message.data = (data_t *)malloc(sizeof(data_t));
	memcpy(message.data, state->dvs[get_current_node()], sizeof(data_t));
	message.size = sizeof(message.data);

	// Send distance vector to neighbors.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		if (get_link_cost(node) < COST_INFINITY && node != get_current_node()) {
			send_message(node, message);
		}
	}

	return state;
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
	fprintf(stderr, "[t=%d]: Node %d received a link change notification from node %d to cost %d.\n", get_current_time(), get_current_node(), neighbor, new_cost);

	state_t *state = (state_t *)get_state();

	bool changed = false;
	if (new_cost == COST_INFINITY) {
		for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
			if (state->via[node] == neighbor) {
				state->dvs[get_current_node()][node] = COST_INFINITY;
				state->via[node] = -1;
				set_route(node, neighbor, COST_INFINITY);
				changed = true;
			}
		}
	}

	// Recompute distance vector.
	// Iterate over all neighbors.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		if (get_link_cost(node) < COST_INFINITY && node != get_current_node() && node != neighbor) {
			cost_t cost = COST_ADD(get_link_cost(node), state->dvs[node][neighbor]);

			if (new_cost > cost) {
				state->dvs[get_current_node()][neighbor] = cost;
				state->via[neighbor] = node;
				set_route(neighbor, node, state->dvs[get_current_node()][neighbor]);
				changed = true;
			}
		}
	}

	// Update distance vector and reset route with new_cost.
	if (!changed) {
		state->dvs[get_current_node()][neighbor] = new_cost;
		state->via[neighbor] = neighbor;
		set_route(neighbor, neighbor, new_cost);
		changed = true;
	}

	// Create message with distance vector.
	message_t message;
	message.data = (data_t *)malloc(sizeof(data_t));
	memcpy(message.data, state->dvs[get_current_node()], sizeof(data_t));
	message.size = sizeof(message.data);

	// Send message only to neighbors.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		cost_t link_cost = get_link_cost(node);
		if (link_cost < COST_INFINITY && node != get_current_node()) {
			send_message(node, message);
		}
	}

	print_table();
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
	fprintf(stderr, "[t=%d]: Node %d received a message from node %d.\n", get_current_time(), get_current_node(), sender);
	state_t *state = (state_t *)get_state();

	// Copy distance vector from message.
	data_t *data = (data_t *)message.data;
	memcpy(state->dvs[sender], data->dv, sizeof(data->dv));

	// Recompute distance vector.
	bool changed = 0;
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {

		if (node == get_current_node()) {
			continue;
		}

		cost_t new_cost = COST_ADD(get_link_cost(sender), state->dvs[sender][node]);
		if (state->dvs[get_current_node()][node] > new_cost) {
			state->dvs[get_current_node()][node] = new_cost;
			state->via[node] = sender;
			set_route(node, sender, state->dvs[get_current_node()][node]);
			changed = 1;
		} else if (state->via[node] == sender) {
			if (state->dvs[get_current_node()][node] != new_cost) {
				state->dvs[get_current_node()][node] = new_cost;
				state->via[node] = sender;
				set_route(node, sender, state->dvs[get_current_node()][node]);
				changed = 1;
			}
		}
	}

	// Create message with distance vector.
	message_t new_message;
	new_message.data = (data_t *)malloc(sizeof(data_t));
	memcpy(new_message.data, state->dvs[get_current_node()], sizeof(data_t));
	new_message.size = sizeof(new_message.data);

	// Send message only to neighbors.
	if (changed) {
		for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
			cost_t link_cost = get_link_cost(node);
			if (link_cost < COST_INFINITY && node != get_current_node()) {
				send_message(node, new_message);
			}
		}
	}

	print_table();
}
