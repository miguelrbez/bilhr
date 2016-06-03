#include "cmac.cpp"
#include <stdio.h>

int main(int argc, char const *argv[])
{
	CMAC cmac_t = CMAC(2, 3, 10, 0.00002);
	printf("Loading CMAC successfull!\n");
	return 0;
}
