#include "../src/mlp.h"
#include <iostream>


int main(int argc, char *argv[])
{
	MLP mlp = MLP(2, 8, 2, 0.1, 0.1, 0.3);
	std::cout << "Hello World.\n";
}