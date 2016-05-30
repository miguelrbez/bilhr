#include "mlp.h"

// name of the file
char fileName[] = "weights.txt";
char mseDataName[] = "task2_error.txt";

MLP::MLP(int n, int p, int q,
		double alpha, double beta, double mse_threshold, int max_interactions)
{
	/*	Do some input parsing	*/
	if (n < 1)
		throw std::out_of_range("At least 1 input neuron is required.");
	if (p < 1)
		throw std::out_of_range("At least 1 hidden neuron is required.");
	if (q < 1)
		throw std::out_of_range("At least 1 output neuron is required.");
	if (alpha <= 0 || alpha >= 1)
		throw std::out_of_range("Alpha needs to be in the range ]0, 1[.");
	if (beta <= 0 || beta >= 1)
		throw std::out_of_range("Beta needs to be in the range ]0, 1[.");
	if (mse_threshold <= 0)
		throw std::out_of_range("The MSE threshold needs to be true positive.");
	n_ = n;
	p_ = p;
	q_ = q;
	alpha_ = alpha;
	beta_ = beta;
	mse_threshold_ = mse_threshold;
	/*	Initialize arrays and variables	*/
	nr_samples_ = 0;
	max_iterations_ = max_interactions;
	initialize_neurons();
	initialize_weights();
}

MLP::~MLP()
{

}

void MLP::add_sample(std::vector<double> input, std::vector<double> output)
{
	if (input.size() != n_)
		throw std::length_error("The length of the input vector is not equal to the number of input neurons.");
	if (output.size() != q_)
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

void saveMSE(double mse)
{
	std::ofstream data;
	data.open(mseDataName, std::ios::app);
	data << mse << "\n";
	data.close();
}

void MLP::train()
{
	uint iteration = 0;
	double mse = 0;
	do {
		iteration++;
		for (int s = 0; s < nr_samples_; s++) {
			calc_fwd_propagation(train_input_[s]);
			calc_bwd_propagation(train_output_[s]);
			adjust_weights(train_input_[s]);
		}
		mse = calc_mse();

		// saving mse
		if(iteration % 200 == 0)
			saveMSE(mse);

	} while( mse > mse_threshold_ && iteration < max_iterations_);
	printf("Training finished after %u out of %u maximum iterations with %d samples.\n", iteration, max_iterations_, nr_samples_);
	if ( mse < mse_threshold_)
		printf("Training succeeded! (MSE = %2.5f\n", mse);
	else
		printf("Training failed... MSE = %2.5f > %2.5f (MSE threshold)\n", mse, mse_threshold_);
}

std::vector<double> MLP::evaluate(std::vector<double> input)
{
	calc_fwd_propagation(input);
	return c_;
}

void MLP::adjust_weights(std::vector<double> input)
{
	/*	Adjust weights for output neurons	*/
	for (int on = 0; on < q_; on++)
		for (int hn = 0; hn < p_; hn++)
			w_[on][hn] += d_[on] * b_[hn];
	/*	Adjust weights for input neurons	*/
	for (int hn = 0; hn < p_; hn++)
		for (int in = 0; in < n_; in++)
			v_[hn][in] += beta_ * input[in] * e_[hn];
	/*	Adjust hidden weights for input neurons		*/
	for (int hn = 0; hn < p_; hn++)
		v_bias_[hn] += beta_ * e_[hn];
	/*	Adjust hidden weights for output neurons	*/
	for (int on = 0; on < q_; on++)
		w_bias_[on] += alpha_ * d_[on];
}

void MLP::calc_bwd_propagation(std::vector<double> output)
{
	double s;
	for (int on = 0; on < q_; on++)
		d_[on] = c_[on] * (1 - c_[on]) * (output[on] - c_[on]);
	for (int hn = 0; hn < p_; hn++) {
		s = 0;
		for (int on = 0; on < q_; on++)
			s += w_[on][hn] * d_[on];
		e_[hn] = b_[hn] * (1 - b_[hn]) * s;
	}
}

void MLP::calc_fwd_propagation(std::vector<double> input)
{
	double s;
	/*	Forward propagate for the hidden layer	*/
	for (int hn = 0; hn < p_; hn++) {
		s = 0;
		for (int in = 0; in < n_; in++)
			s += input[in] * v_[hn][in];
		b_[hn] = sigmoid(s + v_bias_[hn]);
	}
	/*	Forward propagate for the output layer	*/
	for (int on = 0; on < q_; on++) {
		s = 0;
		for (int hn = 0; hn < p_; hn++)
			s += b_[hn] * w_[on][hn];
		c_[on] = sigmoid(s + w_bias_[on]);
	}
}

double MLP::calc_mse()
{
	double mse = 0;
	for (int s = 0; s < nr_samples_; s++)
		for (int on = 0; on < q_; on++)
			mse += pow(train_output_[s][on] - c_[on], 2.0);
	return mse / (2 * nr_samples_ * q_);
}

void MLP::initialize_neurons()
{
	b_ = std::vector<double> (p_);
	c_ = std::vector<double> (q_);
	d_ = std::vector<double> (q_);
	e_ = std::vector<double> (p_);
}

void MLP::initialize_weights()
{
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-1.0, +1.0);

	w_bias_.clear();
	v_.clear();
	v_bias_.clear();
	w_.clear();

	for (int i = 1; i <= q_; i++)
		w_bias_.push_back(distribution(generator));
	for (int i = 1; i <= n_; i++)
		v_bias_.push_back(distribution(generator));
	for (int hn = 1; hn <= p_; hn++) {
		std::vector<double> v;
		for (int in = 1; in <= n_; in++)
			v.push_back(distribution(generator));
		v_.push_back(v);
	}
	for (int on = 1; on <= q_; on++) {
		std::vector<double> v;
		for (int hn = 1; hn <= p_; hn++)
			v.push_back(distribution(generator));
		w_.push_back(v);
	}
}

double MLP::sigmoid(double value)
{
	/*	Fast approximation of the sigmoid function	*/
	// return value / (1 + abs(value));
	/*	Exact sigmoid function	*/
	return 1 / (1 + exp(-value));
}


void MLP::save()
{
	std::cout << "saving values into file...";
	std::ofstream file;
	file.open(fileName);

	std::vector< std::vector<double> >::const_iterator row;
  std::vector<double>::const_iterator col;

	for(row = v_.begin(); row != v_.end(); ++row)
	{
		// writing the size of the line as the first element encoding the matrix
		file << "10000\t" << row->size() << "\t";
		for (col = row->begin(); col != row->end(); ++col)
    {
			// writing value
			file << *col << "\t";
		}
		file << "\n";
	}


	file << "20000\t" << v_bias_.size() << "\t";
	for(int i = 0; i < v_bias_.size(); i++)
	{
		file << v_bias_[i] << "\t";
	}
	file << "\n";


	for(row = w_.begin(); row != w_.end(); ++row)
	{
		// writing the size of the line as the first element
		file << "30000\t" << row->size() << "\t";
		for (col = row->begin(); col != row->end(); ++col)
		{
			// writing value
			file << *col << "\t";
		}
		file << "\n";
	}


	file << "40000\t" << w_bias_.size() << "\t";
	for(int i = 0; i < w_bias_.size(); i++)
	{
		file << w_bias_[i] << "\t";
	}
	file << "\n";

	file.close();

	std::cout << "done\n";
}

void MLP::load()
{
	double tmp;
	int items;

	std::cout << "loading values from file...";
	std::ifstream file(fileName);
	//
	// load weights_input
	while(!file.eof())
	{
		std::vector<double> vec;

		// loading encoded vector name
		file >> tmp;

		if(tmp == 10000)
		{
			// loading size of a line
			file >> items;

			// writing line into vector
			for(int i = 0; i < items; i++)
			{
				file >> tmp;
				vec.push_back(tmp);
			}
			// writing values into vector
			v_.push_back(vec);
		}
		else if(tmp == 20000)
		{
			// loading size of a line
			file >> items;

			// writing line into vector
			for(int i = 0; i < items; i++)
			{
				file >> tmp;
				v_bias_.push_back(tmp);
			}
		}
		else if(tmp == 30000)
		{
			// loading size of a line
			file >> items;

			// writing line into vector
			for(int i = 0; i < items; i++)
			{
				file >> tmp;
				vec.push_back(tmp);
			}
			w_.push_back(vec);
		}
		else if(tmp == 40000)
		{
			// loading size of a line
			file >> items;

			// writing line into vector
			for(int i = 0; i < items; i++)
			{
				file >> tmp;
				w_bias_.push_back(tmp);
			}
		}
	}

	std::cout << "done\n";
}
