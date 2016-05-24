#include "mlp.h"

MLP::MLP(int nr_input_neurons, int nr_hidden_neurons, int nr_output_neurons,
		double alpha, double beta, double mse_threshold)
{
	/*	Do some input parsing	*/
	if (nr_input_neurons < 1)
		throw std::out_of_range("At least 1 input neuron is required.");
	if (nr_hidden_neurons < 1)
		throw std::out_of_range("At least 1 hidden neuron is required.");
	if (nr_output_neurons < 1)
		throw std::out_of_range("At least 1 output neuron is required.");
	if (alpha <= 0 || alpha >= 1)
		throw std::out_of_range("Alpha needs to be in the range ]0, 1[.");
	if (beta <= 0 || beta >= 1)
		throw std::out_of_range("Beta needs to be in the range ]0, 1[.");
	if (mse_threshold <= 0)
		throw std::out_of_range("The MSE threshold needs to be true positive.");
	nr_input_neurons_ = nr_input_neurons;
	nr_hidden_neurons_ = nr_hidden_neurons;
	nr_output_neurons_ = nr_output_neurons;
	alpha_ = alpha;
	beta_ = beta;
	mse_threshold_ = mse_threshold;
}

MLP::~MLP()
{

}

void MLP::add_sample(std::vector<double> input, std::vector<int> output)
{
	// TODO: do vector length checking
	// TODO: implement
}

void MLP::train()
{

}

std::vector<double> MLP::evaluate(std::vector<double> input)
{
	std::vector<double> output;
	return output;
}