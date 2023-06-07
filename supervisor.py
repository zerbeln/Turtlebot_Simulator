import numpy as np
import math


class Supervisor:
    def __init__(self, n_inp=9, n_out=360, n_hid=30):
        self.turtlebot_scan_dist = 3.5  # Turtlebot's scan distance (meters)
        self.n_inputs = n_inp  # Number of nodes in input layer
        self.n_outputs = n_out  # Number of nodes in output layer
        self.n_hidden = n_hid  # Number of nodes in hidden layer
        self.weights = {}
        self.input_layer = np.zeros(self.n_inputs)
        self.hidden_layer = np.zeros(self.n_hidden)
        self.output_layer = np.zeros(self.n_outputs)

    def scan_environment(self, turtlebot, target, hazard):
        """
        Supervisor collects information about environment
        """
        # Turtlebot data
        self.input_layer[0] = turtlebot[0]  # Turtlebot x position
        self.input_layer[1] = turtlebot[1]  # Turtlebot y position
        self.input_layer[2] = turtlebot[2]  # Turtlebot heading

        # Target data
        self.input_layer[3] = target[0]  # Target center x-coordinate
        self.input_layer[4] = target[1]  # Target center y-coordinate
        self.input_layer[5] = target[2]  # Target radius

        # Hazard sata
        self.input_layer[6] = hazard[0]  # Hazard center x-coordinate
        self.input_layer[7] = hazard[1]  # Hazard center y-coordinate
        self.input_layer[8] = hazard[2]  # Hazard radius

    # Supervisor NN ------------------------------------------------------------------------------------------------
    def get_weights(self, nn_weights):
        """
        Apply chosen network weights to supervisor's neural network
        """
        self.weights['l1'] = nn_weights['L1'][0:self.n_inputs, :]
        self.weights['l1_bias'] = nn_weights['L1'][self.n_inputs, :]  # Biasing weights
        self.weights['l2'] = nn_weights['L2'][0:self.n_hidden, :]
        self.weights['l2_bias'] = nn_weights['L2'][self.n_hidden, :]  # Biasing weights

    def get_nn_outputs(self):
        """
        Run NN to generate counterfactual for worker agents
        """
        self.hidden_layer = np.dot(self.input_layer, self.weights['l1']) + self.weights['l1_bias']
        self.hidden_layer = self.sigmoid(self.hidden_layer)

        self.output_layer = np.dot(self.hidden_layer, self.weights['l2']) + self.weights['l2_bias']
        self.output_layer = self.sigmoid(self.output_layer)

    def run_supervisor_nn(self, turtlebot, target, hazard):
        """
        Run neural network using state information, return counterfactual state
        """
        self.scan_environment(turtlebot, target, hazard)
        self.get_nn_outputs()

        # Convert NN outputs to a LiDAR distance
        counterfactual = np.multiply(self.output_layer, -1.5*self.turtlebot_scan_dist)

        return counterfactual

    # Activation Functions -------------------------------------------------------------------------------------------
    def tanh(self, inp):  # Tanh function as activation function
        """
        tanh neural network activation function
        """
        tanh = (2 / (1 + np.exp(-2 * inp))) - 1

        return tanh

    def sigmoid(self, inp):  # Sigmoid function as activation function
        """
        sigmoid neural network activation function
        """
        sig = 1 / (1 + np.exp(-inp))

        return sig
