#include "cmac.h"
using namespace std;

CMAC::CMAC(int number_output_neurons, int field_size, int resolution)
{
	if (number_output_neurons < 1)
		throw std::out_of_range("At least 1 output neuron is needed.");
	if (field_size < 2)
		throw std::out_of_range("The field size must be at least 2x2.");
	if (resolution < 2)
		throw std::out_of_range("The input resolution must be at least 2.");
	n_x_ = number_output_neurons;
	n_a_ = field_size;
	n_s_ = 0;
	resolution_ = resolution;
	RFpos_ = gen_static_perceptive_field(n_a_);
	initialize_weights();
}

CMAC::~CMAC()
{

}

void CMAC::set_alpha(double alpha)
{
	// TODO: implement set_alpha
}

void CMAC::set_max_train_iterations(int max_iterations)
{
	if (max_iterations <= 1)
		throw std::out_of_range("max_iterations can not be negative or 0.");
	max_iterations_ = max_iterations;
}

void CMAC::train()
{
	// TODO: implement train
}

void CMAC::add_sample(std::vector<double> input, std::vector<double> output)
{
	if (input.size() != 2)
		throw std::length_error("The input data sample must have length 2.");
	if (input[0] < 0 || input[0] > 1 || input[1] < 0 || input[1] > 1)
		throw std::out_of_range("The input data sample values must be in the range [0, 1].");
	if (output.size() != n_x_)
		throw std::length_error("The output data must have the length of output neurons.");
	i_.push_back(input);
	t_.push_back(output);
	n_s_++;
}

std::vector<double> CMAC::evaluate(std::vector<double> input)
{
	// TODO: check, if the input is within the range [0, 1]
	// TODO: implement evaluate
}

double CMAC::calc_mse()
{
	// TODO: implement calc_mse
}

std::vector< std::pair<int, int> > CMAC::calc_activated_neurons(std::vector<double> input, std::vector< std::pair<int, int> > pf)
{
	vector< std::pair<int, int> > position;
	for (int r = 0; r < pf.size(); r++)
	{

		pair<int, int> coord;
		for (int c = 0; c < input.size(); c++)
		{
			int input_index = round(input[c]*resolution_);
			int shift_amount = n_a_ - input_index % n_a_;
			pair<int, int> local_coord;
			if(c == 0){
				local_coord.first = (shift_amount - pf[r].first) % n_a_;
				coord.first = input_index + local_coord.first;
			} else if(c == 1){
				local_coord.second = (shift_amount - pf[r].second) % n_a_;
				coord.second = input_index + local_coord.second;
			}
		}
		position.push_back(coord);
		cout << "coord " << coord.first << " " << coord.second << endl;
	}
	return position;
}

void CMAC::adjust_weights()
{
	// TODO: implement adjust_weights

}

std::vector< std::pair<int, int> > CMAC::gen_static_perceptive_field(int field_size)
{
	std::vector< std::pair<int, int> > pf;
	switch (field_size) {
		case 3:	pf.push_back(std::make_pair(0, 2));  // o o x
				pf.push_back(std::make_pair(1, 0));  // x o o
				pf.push_back(std::make_pair(2, 1));  // o x o
				break;
		case 5:	pf.push_back(std::make_pair(0, 3));  // o o o x o
				pf.push_back(std::make_pair(1, 0));  // x o o o o
				pf.push_back(std::make_pair(2, 2));  // o o x o o
				pf.push_back(std::make_pair(3, 4));  // o o o o x
				pf.push_back(std::make_pair(4, 1));  // o x o o o
				break;
		default:	throw std::out_of_range("Field size is not supported for static perceptive field generation.");
	}
	return pf;
}

std::vector< std::pair<int, int> > CMAC::gen_random_perceptive_field(int field_size)
{
	// TODO: implement gen_random_perceptive_field
}

void CMAC::initialize_weights()
{
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-1.0, +1.0);

	w_ = std::vector< std::vector<double> > (n_x_);
	std::vector<double> weights (n_a_);
    for (int i = 0; i < n_x_; i++) {
	    for (int j = 0; j < n_a_; j++)
	    	weights[j] = distribution(generator);
	    w_[i] = weights;
    }
}
