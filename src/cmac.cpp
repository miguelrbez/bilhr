#include "cmac.h"

CMAC::CMAC(int number_output_neurons, int field_size, int resolution)
{

}

void CMAC::set_alpha(double alpha)
{
	// TODO: implement set_alpha
}

void CMAC::add_sample(std::vector<double> input, std::vector<double> output)
{
	// TODO: check, if the input is within the range [0, 1]
	// TODO: implement add_sample
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

void CMAC::adjust_weights()
{
	// TODO: implement adjust_weights
}
