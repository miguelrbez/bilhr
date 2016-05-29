#include "../src/mlp.cpp"
#include <iostream>


int main(int argc, char *argv[])
{
//    MLP mlp (3, 6, 2, 0.6, 0.5, 0.4);
//	std::cout << "Hello World.\n";
//	std::vector<double> sample_input = {0.1, -0.4, 5.1};
//	std::vector<double> sample_output = {0.2, -0.81, 8};
//	mlp.add_sample(sample_input, sample_output);

    // task 2 XOR
    MLP mlp(2, 2, 1, 0.05, 0.05, 0.00002);
    mlp.set_max_iterations(20000);
    
    // input sample
    vector<double> inputSample;
    vector<double> outputSample;

    // XOR - first line
    inputSample.push_back(0.0001);
    inputSample.push_back(0.0001);
    outputSample.push_back(0.0001);
    mlp.add_sample(inputSample, outputSample);

    // clear vector
    inputSample.clear();
    outputSample.clear();

    // XOR - second line
    inputSample.push_back(0.9999);
    inputSample.push_back(0.0001);
    outputSample.push_back(0.9999);
    mlp.add_sample(inputSample, outputSample);

    // clear vector
    inputSample.clear();
    outputSample.clear();

    // XOR - third line
    inputSample.push_back(0.0001);
    inputSample.push_back(0.9999);
    outputSample.push_back(0.9999);
    mlp.add_sample(inputSample, outputSample);

    // clear vector
    inputSample.clear();
    outputSample.clear();

    // XOR - forth line
    inputSample.push_back(0.9999);
    inputSample.push_back(0.9999);
    outputSample.push_back(0.0001);
    mlp.add_sample(inputSample, outputSample);

    mlp.train();

    mlp.save();
    mlp.load();
}
