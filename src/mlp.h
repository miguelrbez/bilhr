#ifndef MLP_H
#define MLP_H

#include <vector>
#include <exception>
#include <stdexcept>
#include <random>

class MLP
{
public:

	/**
	 * @brief      Initializes the Multi Layer Perceptron (MLP) Neural 
	 * Network (NN).
	 *
	 * @param[in]  nr_input_neurons   The number of input neurons
	 * @param[in]  nr_hidden_neurons  The number of hidden neurons
	 * @param[in]  nr_output_neurons  The number of output neurons
	 * @param[in]  alpha              Learning rate for output weights
	 * @param[in]  beta               Learning rate for input weights
	 * @param[in]  mse_threshold      The Mean Square Error (MSE) threshold
	 */
	MLP(int nr_input_neurons, int nr_hidden_neurons, int nr_output_neurons,
		double alpha, double beta, double mse_threshold);
	~MLP();

	/**
	 * @brief      Adds a sample. input and output must of length
	 * nr_input_neurons and nr_output_neurons.
	 *
	 * @param[in]  input   contains the input values for this sample.
	 * @param[in]  output  contains the output values for this sample.
	 */
	void add_sample(std::vector<double> input, std::vector<double> output);
	void train();
	std::vector<double> evaluate(std::vector<double> input);

private:
	/**
	 * Holds the number of samples. The running index is k = 1..m
	 */
	int nr_samples_;
	/**
	 * Holds the number of input neurons. The running index is h = 1..n
	 */
	int nr_input_neurons_;
	/**
	 * Holds the number of hidden neurons. Only one layer is supported. The
	 * running index is i = 1..p
	 */
	int nr_hidden_neurons_;
	/**
	 * Holds the number of output neurons. The running index is j = 1..g
	 */
	int nr_output_neurons_;
	/**
	 * 2-dimensional vector containing the weights for the input layer. Each
	 * message from the input layer neurons to the hidden layer neurons will
	 * be weighted with the corresponding weight. It is denoted by the variable
	 * v in the literature.
	 * The array has the size nr_input_neurons_ x nr_hidden_neurons_.
	 */
	std::vector< std::vector<double> > weights_input_;
	/**
	 * Weights of the input biases. It is denoted by the variable theta in the
	 * literature.
	 * The array has the size nr_input_neurons_.
	 */
	std::vector<double> weights_input_bias_;
	/**
	 * 2-dimensional vector containing the weights for the output layer. Each
	 * message from the hidden layer neurons to the output layer neurons will
	 * be weighted with the corresponding weight. It is denoted by the variable
	 * w in the literature.
	 * The array has the size nr_hidden_neurons_ x nr_output_neurons_.
	 */
	std::vector< std::vector<double> > weights_output_;
	/**
	 * Weights for the bias of the output layer neurons. Denoted by phi in the 
	 * literature.
	 * The array has the length nr_output_neurons_.
	 */
	std::vector<double> weights_output_bias_;
	/**
	 * This vector will be filled with the input of the training data. It has
	 * length of the number of training samples nr_samples_. Each element 
	 * contains one input vector for this specific training sample. It has the
	 * length nr_input_neurons_;
	 */
	std::vector< std::vector<double> > train_input_;
	/**
	 * This vector will be filled with the output of the training data. It has
	 * length of the number of training samples nr_samples_. Each element 
	 * contains one output vector for this specific training sample. It has the
	 * length nr_output_neurons_;
	 */
	std::vector< std::vector<double> > train_output_;
	/**
	 * This variable holds the values of the hidden neurons. It has the length
	 * nr_hidden_neurons_. It can be used for the training of the FFNN or the
	 * evaluation of an input.
	 */
	std::vector<double> b_;
	/**
	 * This variable holds the values of the output neurons. It has the length
	 * nr_output_neurons_. It can be used for the training of the FFNN or the
	 * evaluation of an input.
	 */
	std::vector<double> c_;
	/**
	 * This variable is used for the back-propagation algorithm and represents 
	 * the output layer. It has the length nr_output_neurons_.
	 */
	std::vector<double> d_;
	/**
	 * This variable is used for the back-propagation algorithm and represents 
	 * the hidden layer. It has the length nr_hidden_neurons_.
	 */
	std::vector<double> e_;

	double alpha_;
	double beta_;
	double mse_threshold_;

	void initialize_neurons();
	void initialize_weights();
};



#endif
