#include "cmac.cpp"
#include <stdio.h>
#include <fstream>
#include <iostream>

using namespace std;

vector< vector<double> > inputData;
vector< vector<double> > outputData;

void loadData()
{
  // load data from file into array
  cout << "loading data for training... \n";
	char path[] = "../tutorials/tutorial4/data.txt";
  ifstream dataFile(path);
  while(!dataFile.eof())
  {
    vector<double> input;
    vector<double> output;
    double tmp;

    // load pic x
    dataFile >> tmp;
    input.push_back(tmp);
    // load pic y
    dataFile >> tmp;
    input.push_back(tmp);

    // load input sample into vector
    inputData.push_back(input);

    // load first shoulder joint
    dataFile >> tmp;
    output.push_back(tmp);
    // load second shouder joint
    dataFile >> tmp;
    output.push_back(tmp);

    // load output sample into vector
    outputData.push_back(output);
  }
  cout << "loading data done\n";
}

int main(int argc, char const *argv[])
{
	CMAC cmac_case_a = CMAC(2, 3, 50, 0.00002);
	CMAC cmac_case_b = CMAC(2, 3, 50, 0.00002);

	// 150 samples are in the file
	loadData();
	int samplesCaseA = 75;
	int samplesCaseB = 150;

	// set alpha
	cmac_case_a.set_alpha(0.005);

	// case A
	for(int i = 0; i < samplesCaseA; i++)
		cmac_case_a.add_sample(inputData[i], outputData[i]);

	cmac_case_a.set_max_train_iterations(200000);
	cmac_case_a.train();

	// case B
	// for(int i = 0; i < samplesCaseB; i++)
	// 	cmac_case_b.add_sample(inputData[i], outputData[i]);
	//
	// cmac_case_b.set_max_train_iterations(200000);
	// cmac_case_b.train();

	printf("Loading CMAC successfull!\n");
	return 0;
}
