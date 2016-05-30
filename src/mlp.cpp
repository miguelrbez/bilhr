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
	max_iterations_ = 1000;
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
	for (int i = 0; i < input.size(); i++)
        if (input[i] < -1.0 || input[i] > +1.0)
			throw std::out_of_range("All elements in INPUT need to be in the range [-1.0, +1.0].");
	for (int i = 0; i < output.size(); i++)
        if (output[i] < -1.0 || output[i] > +1.0)
			throw std::out_of_range("All elements in OUTPUT need to be in the range [-1.0, +1.0].");
	train_input_.push_back(input);
	train_output_.push_back(output);
	nr_samples_++;
}

void MLP::set_max_iterations(int max_iterations)
{
	if (max_iterations <= 1)
		throw std::out_of_range("max_iterations can not be negative or 0.");
	max_iterations_ = max_iterations;
}

void MLP::train()
{
	uint iteration = 0;
	double mse = 0;
	std::vector< std::vector<double> > c_eval (nr_samples_);
	do {
		iteration++;
		for (int s = 0; s < nr_samples_; s++) {
			c_eval[s] = calc_fwd_propagation(train_input_[s]);
			calc_bwd_propagation(train_output_[s]);
			adjust_weights(train_input_[s]);
		}
		mse = calc_mse();
		if (iteration < 100 || iteration % 1000 == 0)
			printf("Iteration %7d, MSE = %2.5f\n", iteration, mse);
	} while( mse > mse_threshold_ && iteration < max_iterations_);
	printf("Training finished after %u out of %u maximum iterations with %d samples.\n", iteration, max_iterations_, nr_samples_);
	if ( mse < mse_threshold_)
		printf("Training succeeded! (MSE = %2.5f)\n", mse);
	else
		printf("Training failed... MSE = %2.5f > %2.5f (MSE threshold)\n", mse, mse_threshold_);
}

std::vector<double> MLP::evaluate(std::vector<double> input)
{
	return calc_fwd_propagation(input);
}

void MLP::adjust_weights(std::vector<double> input)
{
	/*	Adjust weights for output neurons	*/
	for (int on = 0; on < nr_output_neurons_; on++)
		for (int hn = 0; hn < nr_hidden_neurons_; hn++)
			weights_output_[on][hn] += d_[on] * b_[hn];
	/*	Adjust weights for input neurons	*/
	for (int hn = 0; hn < nr_hidden_neurons_; hn++)
		for (int in = 0; in < nr_input_neurons_; in++)
			weights_input_[hn][in] += beta_ * input[in] * e_[hn];
	/*	Adjust hidden weights for input neurons		*/
	for (int hn = 0; hn < nr_hidden_neurons_; hn++)
		weights_input_bias_[hn] += beta_ * e_[hn];
	/*	Adjust hidden weights for output neurons	*/
	for (int on = 0; on < nr_output_neurons_; on++)
		weights_output_bias_[on] += alpha_ * d_[on];
}

void MLP::calc_bwd_propagation(std::vector<double> output)
{
	double s;
	for (int on = 0; on < nr_output_neurons_; on++)
		d_[on] = c_[on] * (1.0 - c_[on]) * (output[on] - c_[on]);
	for (int hn = 0; hn < nr_hidden_neurons_; hn++) {
		s = 0;
		for (int on = 0; on < nr_output_neurons_; on++)
			s += weights_output_[on][hn] * d_[on];
		e_[hn] = b_[hn] * (1.0 - b_[hn]) * s;
	}
}

std::vector<double> MLP::calc_fwd_propagation(std::vector<double> input)
{
	double s;
	/*	Forward propagate for the hidden layer	*/
	for (int hn = 0; hn < nr_hidden_neurons_; hn++) {
		s = 0;
		for (int in = 0; in < nr_input_neurons_; in++)
			s += input[in] * weights_input_[hn][in];
		b_[hn] = sigmoid(s + weights_input_bias_[hn]);
	}
	/*	Forward propagate for the output layer	*/
	for (int on = 0; on < nr_output_neurons_; on++) {
		s = 0;
		for (int hn = 0; hn < nr_hidden_neurons_; hn++)
			s += b_[hn] * weights_output_[on][hn];
		c_[on] = sigmoid(s + weights_output_bias_[on]);
	}
	return c_;
}

double MLP::calc_mse(std::vector< std::vector<double> > c_eval)
{
	double mse = 0.0;
	for (int s = 0; s < nr_samples_; s++)
		for (int on = 0; on < nr_output_neurons_; on++)
			mse += pow(train_output_[s][on] - c_eval[s][on], 2.0);
	return mse / (2.0 * nr_samples_ * nr_output_neurons_);
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

    for (int i = 0; i < nr_output_neurons_; i++)
		weights_output_bias_.push_back(distribution(generator));
    for (int i = 0; i < nr_input_neurons_; i++)
		weights_input_bias_.push_back(distribution(generator));
    for (int hn = 0; hn < nr_hidden_neurons_; hn++) {
		std::vector<double> v;
        for (int in = 0; in < nr_input_neurons_; in++)
			v.push_back(distribution(generator));
		weights_input_.push_back(v);
	}
    for (int on = 0; on < nr_output_neurons_; on++) {
		std::vector<double> v;
        for (int hn = 0; hn < nr_hidden_neurons_; hn++)
			v.push_back(distribution(generator));
		weights_output_.push_back(v);
	}
}

double MLP::sigmoid(double value)
{
	/*	Fast approximation of the sigmoid function	*/
	// return value / (1 + abs(value));
	/*	Exact sigmoid function	*/
	return 1.0 / (1.0 + exp(-value));
}


void MLP::save(std::string file_name)
{

}

void MLP::load(std::string file_name)
{

}

