#include "../src/mlp.cpp"
#include <iostream>


int main(int argc, char *argv[])
{
    MLP mlp (3, 6, 2, 0.6, 0.5, 0.4);
	std::cout << "Hello World.\n";
	std::vector<double> sample_input = {0.1, -0.4, 5.1};
	std::vector<double> sample_output = {0.2, -0.81, 8};
	mlp.add_sample(sample_input, sample_output);
}
