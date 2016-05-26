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
	/*	Initialize arrays and variables	*/
	nr_samples_ = 0;
	initialize_neurons();
	initialize_weights();
}

MLP::~MLP()
{

}

void MLP::add_sample(std::vector<double> input, std::vector<double> output)
{
	if (input.size() != nr_input_neurons_)
		throw std::length_error("The length of the input vector is not equal to the number of input neurons.");
	if (output.size() != nr_output_neurons_)
		throw std::length_error("The length of the output vector is not equal to the number of output neurons.");
	// TODO: check, if input and output elements are in the range [-1.0, +1.0]
	train_input_.push_back(input);
	train_output_.push_back(output);
	nr_samples_++;
}

void MLP::train()
{

}

std::vector<double> MLP::evaluate(std::vector<double> input)
{
	// TODO: implement evaluate method
	std::vector<double> output;
	return output;
}

void MLP::initialize_neurons()
{
	b_ = std::vector<double> (nr_hidden_neurons_);
	c_ = std::vector<double> (nr_output_neurons_);
	d_ = std::vector<double> (nr_output_neurons_);
	e_ = std::vector<double> (nr_hidden_neurons_);
}

void MLP::initialize_weights()
{
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-1.0, +1.0);

	weights_output_bias_.clear();
	weights_input_.clear();
	weights_input_bias_.clear();
	weights_output_.clear();

	for (int i = 1; i < nr_output_neurons_; i++)
		weights_output_bias_.push_back(distribution(generator));
	for (int i = 1; i < nr_input_neurons_; i++)
		weights_input_bias_.push_back(distribution(generator));
	for (int hn = 1; hn < nr_hidden_neurons_; hn++) {
		std::vector<double> v;
		for (int in = 1; in < nr_input_neurons_; in++)
			v.push_back(distribution(generator));
		weights_input_.push_back(v);
	}
	for (int on = 1; on < nr_output_neurons_; on++) {
		std::vector<double> v;
		for (int hn = 1; hn < nr_hidden_neurons_; hn++)
			v.push_back(distribution(generator));
		weights_output_.push_back(v);
	}
}