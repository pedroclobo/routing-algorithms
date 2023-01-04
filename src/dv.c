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
	cost_t d[MAX_NODES];
} data_t;

// State format.
typedef struct {
	cost_t d[MAX_NODES];
} state_t;

// Handler for the node to allocate and initialize its state.
void *init_state() {
	state_t *state = (state_t *)malloc(sizeof(state_t));

	// Initialize distance vector.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		state->d[node] = get_link_cost(node);
	}

	return state;
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
	state_t *state = (state_t *)get_state();

	state->d[neighbor] = new_cost;
	set_route(neighbor, neighbor, new_cost);

	// Create message with distance vector.
	message_t message;
	message.data = (data_t *)malloc(sizeof(data_t));
	memcpy(message.data, state->d, sizeof(data_t));
	message.size = sizeof(message.data);

	// Send message only to neighbors.
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		cost_t link_cost = get_link_cost(node);
		if (link_cost < COST_INFINITY && node != get_current_node()) {
			send_message(node, message);
		}
	}
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
	assert(sender != get_current_node());

	state_t *state = (state_t *)get_state();
	data_t *data = (data_t *)message.data;

	// Recompute distance vector.
	bool changed = 0;
	for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
		cost_t new_cost = COST_ADD(get_link_cost(sender), data->d[node]);
		if (state->d[node] > new_cost) {
			state->d[node] = new_cost;
			set_route(node, sender, state->d[node]);
			changed = 1;
		}
	}

	// Create message with distance vector.
	message_t new_message;
	new_message.data = (data_t *)malloc(sizeof(data_t));
	memcpy(new_message.data, state->d, sizeof(data_t));
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
}
