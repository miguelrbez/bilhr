#ifndef MLP_H
#define MLP_H

#include <vector>
#include <exception>
#include <stdexcept>
#include <random>
#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>

class MLP
{
public:

	/**
	 * @brief      Initializes the Multi Layer Perceptron (MLP) Neural
	 * Network (NN).
	 *
	 * @param[in]  n   The number of input neurons
	 * @param[in]  p  The number of hidden neurons
	 * @param[in]  q  The number of output neurons
	 * @param[in]  alpha              Learning rate for output weights
	 * @param[in]  beta               Learning rate for input weights
	 * @param[in]  mse_threshold      The Mean Square Error (MSE) threshold
	 */
	MLP(int n, int p, int q,
		double alpha, double beta, double mse_threshold, int max_interactions);
	~MLP();

	/**
	 * @brief      Adds a sample. input and output must of length
	 * n and q.
	 *
	 * @param[in]  input   contains the input values for this sample.
	 * @param[in]  output  contains the output values for this sample.
	 */
	void add_sample(std::vector<double> input, std::vector<double> output);
	void train();
	std::vector<double> evaluate(std::vector<double> input);
	void save();
	void load();

private:
	/**
	 * Holds the number of samples. The running index is k = 1..m
	 */
	int nr_samples_;
	/**
	 * Holds the number of input neurons. The running index is h = 1..n
	 */
	int n_;
	/**
	 * Holds the number of hidden neurons. Only one layer is supported. The
	 * running index is i = 1..p
	 */
	int p_;
	/**
	 * Holds the number of output neurons. The running index is j = 1..g
	 */
	int q_;
	/**
	 * 2-dimensional vector containing the weights for the input layer. Each
	 * message from the input layer neurons to the hidden layer neurons will
	 * be weighted with the corresponding weight. It is denoted by the variable
	 * v in the literature.
	 * The array has the size n_ x p_.
	 */
	std::vector< std::vector<double> > v_;
	/**
	 * Weights of the input biases. It is denoted by the variable theta in the
	 * literature.
	 * The array has the size n_.
	 */
	std::vector<double> v_bias_;
	/**
	 * 2-dimensional vector containing the weights for the output layer. Each
	 * message from the hidden layer neurons to the output layer neurons will
	 * be weighted with the corresponding weight. It is denoted by the variable
	 * w in the literature.
	 * The array has the size p_ x q_.
	 */
	std::vector< std::vector<double> > w_;
	/**
	 * Weights for the bias of the output layer neurons. Denoted by phi in the
	 * literature.
	 * The array has the length q_.
	 */
	std::vector<double> w_bias_;
	/**
	 * This vector will be filled with the input of the training data. It has
	 * length of the number of training samples nr_samples_. Each element
	 * contains one input vector for this specific training sample. It has the
	 * length n_;
	 */
	std::vector< std::vector<double> > train_input_;
	/**
	 * This vector will be filled with the output of the training data. It has
	 * length of the number of training samples nr_samples_. Each element
	 * contains one output vector for this specific training sample. It has the
	 * length q_;
	 */
	std::vector< std::vector<double> > train_output_;
	/**
	 * This variable holds the values of the hidden neurons. It has the length
	 * p_. It can be used for the training of the FFNN or the
	 * evaluation of an input.
	 */
	std::vector<double> b_;
	/**
	 * This variable holds the values of the output neurons. It has the length
	 * q_. It can be used for the training of the FFNN or the
	 * evaluation of an input.
	 */
	std::vector<double> c_;
	/**
	 * This variable is used for the back-propagation algorithm and represents
	 * the output layer. It has the length q_.
	 */
	std::vector<double> d_;
	/**
	 * This variable is used for the back-propagation algorithm and represents
	 * the hidden layer. It has the length p_.
	 */
	std::vector<double> e_;

	double alpha_;
	double beta_;
	double mse_threshold_;

	uint max_iterations_;

	void adjust_weights(std::vector<double> input);
	void calc_bwd_propagation(std::vector<double> output);
	void calc_fwd_propagation(std::vector<double> input);
	double calc_mse();
	void initialize_neurons();
	void initialize_weights();
	double sigmoid(double value);
};



#endif
