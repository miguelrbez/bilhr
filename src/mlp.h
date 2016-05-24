#ifndef MLP_H
#define MLP_H

#include <vector>
#include <exception>

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
	void add_sample(std::vector<double> input, std::vector<int> output);
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
	std::vector<double> weights_input_bias_;	// thetha
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
	 * iterature.
	 * The array has the length nr_output_neurons_.
	 */
	std::vector<double> weights_output_bias_;

	double alpha_;
	double beta_;
	double mse_threshold_;
};



#endif
