#ifndef CMAC_H
#define CMAC_H

#include <vector>

class CMAC
{
public:
	/**
	 * @brief      Constructor of the Cerebellar Model Articulation Controller
	 *             class.
	 *
	 * @param[in]  number_output_neurons  The number of output neurons
	 * @param[in]  field_size             The perceptive field size
	 * @param[in]  resolution             The resolution of the input neurons
	 */
	CMAC(int number_output_neurons, int field_size, int resolution);
	~CMAC();

	/**
	 * @brief      Sets the learning rate value alpha. It has to be in the range
	 *             [0-1].
	 *
	 * @param[in]  alpha  The learning rate alpha
	 */
	void set_alpha(double alpha);
	void add_sample(std::vector<double> input, std::vector<double> output);
	std::vector<double> evaluate(std::vector<double> input);

private:
	double alpha_;
	int n_a_;
	int n_x_;
	int n_v_;
	int resolution_;
	std::vector< std::vector<double> > t_;
	std::vector< std::vector<double> > i_;

	double calc_mse();
	void adjust_weights();
};

#endif
