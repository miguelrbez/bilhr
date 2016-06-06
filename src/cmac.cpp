#include "cmac.h"

CMAC::CMAC(int number_output_neurons, int field_size, int resolution, double mse_threshold)
{
	if (number_output_neurons < 1)
		throw out_of_range("At least 1 output neuron is needed.");
	if (field_size < 2)
		throw out_of_range("The field size must be at least 2x2.");
	if (resolution < 2)
		throw out_of_range("The input resolution must be at least 2.");
	n_x_ = number_output_neurons;
	n_a_ = field_size;
	n_s_ = 0;
	resolution_ = resolution;
	mse_threshold_ = mse_threshold;
	x_.resize(n_x_);
	RFpos_ = gen_static_perceptive_field(n_a_);
	initialize_weights();
}

CMAC::~CMAC()
{

}

void CMAC::set_alpha(double alpha)
{
	if (alpha <= 0.0 || alpha >= 1.0)
		throw out_of_range("The learning rate alpha must be in the range ]0, 1[.");
	alpha_ = alpha;
}

void CMAC::set_max_train_iterations(int max_iterations)
{
	if (max_iterations <= 1)
		throw out_of_range("max_iterations can not be negative or 0.");
	max_iterations_ = max_iterations;
}

void CMAC::train()
{
	uint iteration = 0;
	double mse;
	do {
		iteration++;
		mse = 0.0;
		for (int s = 0; s < n_s_; s++) {
			/**
				* Evaluate the CMAC network for the current sample s. The result is
				* directly used to calculate the mean square error (MSE). The MSE is
				* normalized with respect to the number of samples.
				*/
			mse += calc_mse(evaluate(i_[s]), s);
			adjust_weights(calc_activated_neurons(i_[s]), s);
		}
		if (iteration % 10000 == 0)
			printf("Iteration %7d, MSE = %2.7f\n", iteration, mse);
	} while( mse > mse_threshold_ && iteration < max_iterations_);
	printf("Training finished after %u out of %u maximum iterations with %d samples.\n", iteration, max_iterations_, n_s_);
	if ( mse < mse_threshold_)
		printf("Training succeeded! (MSE = %2.5f)\n", mse);
	else
		printf("Training failed... MSE = %2.5f > %2.5f (MSE threshold)\n", mse, mse_threshold_);
}

void CMAC::add_sample(vector<double> input, vector<double> output)
{
	verify_input(input);
	if (output.size() != n_x_)
		throw length_error("The output data must have the length of output neurons.");
	i_.push_back(input);
	t_.push_back(output);
	n_s_++;
}

vector<double> CMAC::evaluate(vector<double> input)
{
	verify_input(input);

	// TODO: implement evaluate
	// write to x_!!!
}

double CMAC::calc_mse(vector<double> c_eval, int sample)
{
	double mse = 0.0;
	for (int i = 0; i < n_x_; i++)
		mse += pow(t_[sample][i] - c_eval[i], 2.0);
	return mse / (2.0 * n_s_ * n_x_);
}

vector< pair<int, int> > CMAC::calc_activated_neurons(vector<double> input)
{
	vector< pair<int, int> > position;
	for (int r = 0; r < RFpos_.size(); r++)
	{

		pair<int, int> coord;
		for (int c = 0; c < input.size(); c++)
		{
			int input_index = round(input[c]*resolution_);
			int shift_amount = n_a_ - input_index % n_a_;
			pair<int, int> local_coord;
			if(c == 0){
				local_coord.first = (shift_amount - RFpos_[r].first) % n_a_;
				coord.first = input_index + local_coord.first;
			} else if(c == 1){
				local_coord.second = (shift_amount - RFpos_[r].second) % n_a_;
				coord.second = input_index + local_coord.second;
			}
		}
		position.push_back(coord);
		cout << "coord " << coord.first << " " << coord.second << endl;
	}
	return position;
}

void CMAC::adjust_weights(vector< pair<int, int> > position, int sample)
{
	for(int i = 0; i < n_x_; i++)
		for(int r = 0; r < position.size(); r++)
			// only considering active neurons
			w_[i][position[r].first][position[r].second] += (alpha_ / n_a_) * (t_[sample][i] - x_[i]);
}

vector< pair<int, int> > CMAC::gen_static_perceptive_field(int field_size)
{
	vector< pair<int, int> > pf;
	switch (field_size) {
		case 3:	pf.push_back(make_pair(0, 2));  // o o x
				pf.push_back(make_pair(1, 0));  // x o o
				pf.push_back(make_pair(2, 1));  // o x o
				break;
		case 5:	pf.push_back(make_pair(0, 3));  // o o o x o
				pf.push_back(make_pair(1, 0));  // x o o o o
				pf.push_back(make_pair(2, 2));  // o o x o o
				pf.push_back(make_pair(3, 4));  // o o o o x
				pf.push_back(make_pair(4, 1));  // o x o o o
				break;
		default:	throw out_of_range("Field size is not supported for static perceptive field generation.");
	}
	return pf;
}

vector< pair<int, int> > CMAC::gen_random_perceptive_field(int field_size)
{
	// TODO: implement gen_random_perceptive_field
}

void CMAC::initialize_weights()
{
	default_random_engine generator;
	uniform_real_distribution<double> distribution(-1.0, +1.0);

	// creating vector
	vector<double> w_tmp(resolution_);

	// creating vector of vector
	vector<vector<double> > matrix;
	for(int i = 0; i < resolution_; i++)
		matrix.push_back(w_tmp);

	// creating vector of vector of vector -> space
	for(int i = 0; i < n_x_; i++)
		w_.push_back(matrix);

	// fill cube with random weights
  for (int i = 0; i < n_x_; i++)
		for (int j = 0; j < resolution_; j++)
			for(int k = 0; k < resolution_; k++)
				w_[i][j][k] = distribution(generator);
}

void CMAC::verify_input(vector<double> input)
{
	if (input.size() != 2)
		throw length_error("The input data sample must have length 2.");
	if (input[0] < 0 || input[0] > 1 || input[1] < 0 || input[1] > 1)
		throw out_of_range("The input data sample values must be in the range [0, 1].");
}
