#ifndef CMAC_H
#define CMAC_H

#include <vector>
#include <utility>
#include <stdexcept>
#include <iostream>

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

	/**
	 * @brief      Sets the maximum train iterations.
	 *
	 * @param[in]  max_iterations  The maximum iterations
	 */
	void set_max_train_iterations(int max_iterations);

	/**
	 * @brief      Trains the network by adjusting the weights.
	 */
	void train();

	/**
	 * @brief      Adds a sample. This consists of the input and the desired
	 *             output. The input data has to be normalized.
	 *
	 * @param[in]  input   The normalized input data in the range [0, 1]
	 * @param[in]  output  The output data
	 */
	void add_sample(std::vector<double> input, std::vector<double> output);

	/**
	 * @brief      Evaluates a certain input with the neural network.
	 *
	 * @param[in]  input  The normalized input data in the range [0, 1]
	 *
	 * @return     The calculated output of the network
	 */
	std::vector<double> evaluate(std::vector<double> input);


private:
	double alpha_;
	int max_iterations_;
	/**
	 * Size of the perceptive field
	 */
	int n_a_;
	/**
	 * Number of output neurons.
	 */
	int n_x_;
	/**
	 * Number of neurons in L2.
	 */
	int n_v_;
	/**
	 * Number of samples.
	 */
	int n_s_;
	int resolution_;
	/**
	 * Training output values.
	 */
	std::vector< std::vector<double> > t_;
	/**
	 * Training input values.
	 */
	std::vector< std::vector<double> > i_;
	std::vector< std::pair<double, double> > RFpos_;

	void adjust_weights();
	double calc_mse();
	/**
	 * @brief      Creates a static perceptive field of size field_size by
	 *             field_size.
	 *
	 * @param[in]  field_size  The field size
	 *
	 * @return     Returns a vector of length field_size. Each element contains
	 *             the coordinates of this neuron. The first element is the row,
	 *             the second the column.
	 */
	std::vector< std::pair<double, double> > gen_static_perceptive_field(int field_size);
	std::vector< std::pair<double, double> > gen_random_perceptive_field(int field_size);
};

#endif
