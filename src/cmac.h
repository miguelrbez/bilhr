#ifndef CMAC_H
#define CMAC_H

#include <vector>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <random>
#include <math.h>

using namespace std;

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
	CMAC(int number_output_neurons, int field_size, int resolution, double mse_threshold);
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
	void add_sample(vector<double> input, vector<double> output);

	/**
	 * @brief      Evaluates a certain input with the neural network.
	 *
	 * @param[in]  input  The normalized input data in the range [0, 1]
	 *
	 * @return     The calculated output of the network
	 */
	vector<double> evaluate(vector<double> input);


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
	double mse_threshold_;
	double mse_learning_threshold = 0.0000000001;

	/**
	 output
	 **/
	vector<double> x_;
	/**
	 * Weights of the network. Contains n_x_ weight matrices (2 cascaded vectors).
	 * Each matrix has the dimension resolution_ x resolution_.
	 */
	vector< vector< vector<double> > > w_;
	/**
	 * Training output values.
	 */
	vector< vector<double> > t_;
	/**
	 * Training input values.
	 */
	vector< vector<double> > i_;
	vector< pair<int, int> > RFpos_;

	void adjust_weights(vector< pair<int, int> > position, int sample);
	double calc_mse(vector<double> c_eval, int sample);
	/**
	 * @brief      Calculates the activated neurons.
	 *
	 * @param[in]  input  The input of y1 and y2.
	 *
	 * @return     The activated neurons.
	 */
	vector< pair<int, int> > calc_activated_neurons(vector<double> input);
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
	vector< pair<int, int> > gen_static_perceptive_field(int field_size);
	vector< pair<int, int> > gen_random_perceptive_field(int field_size);
	void initialize_weights();
	void verify_input(vector<double> input);
};

#endif
