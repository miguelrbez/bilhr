#include "cmac.cpp"
#include <stdio.h>

int main(int argc, char const *argv[])
{
	CMAC cmac_case_a = CMAC(2, 3, 50, 0.002);
	CMAC cmac_case_b = CMAC(2, 3, 50, 0.002);
	printf("Loading CMAC successfull!\n");
	return 0;
}
