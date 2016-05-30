#include "../src/mlp.cpp"
#include <iostream>
#include <vector>


int main(int argc, char *argv[])
{
    // task 2 XOR
    MLP mlp(2, 2, 1, 0.05, 0.05, 0.00002);
    mlp.set_max_iterations(200000);
    // input sample
    std::vector< std::vector<double> > input_samples;
    std::vector< std::vector<double> > output_samples;

    // XOR - first line
    input_samples.push_back({0.0001, 0.0001});
    output_samples.push_back({0.0001});
    mlp.add_sample(input_samples.back(), output_samples.back());

    // XOR - second line
    input_samples.push_back({0.9999, 0.0001});
    output_samples.push_back({0.9999});
    mlp.add_sample(input_samples.back(), output_samples.back());

    // XOR - third line
    input_samples.push_back({0.0001, 0.9999});
    output_samples.push_back({0.9999});
    mlp.add_sample(input_samples.back(), output_samples.back());

    // XOR - forth line
    input_samples.push_back({0.9999, 0.9999});
    output_samples.push_back({0.0001});
    mlp.add_sample(input_samples.back(), output_samples.back());

    mlp.train();

    for (int i = 0; i < input_samples.size(); i++)
        printf("Input: %1.4f, %1.4f | Output = %1.4f | Expected Output = %1.4f\n",
            input_samples.at(i)[0], input_samples.at(i)[1],
            mlp.evaluate(input_samples.at(i))[0], output_samples.at(i)[0]);

}
