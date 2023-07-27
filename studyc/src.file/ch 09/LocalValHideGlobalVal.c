#include <stdio.h>
int Add(int val);
int num=1;

int main(void)
{
	int num=5;
	printf("num: %d \n", Add(3));
	printf("num: %d \n", num+9);
	return 0;
}

int Add(int val)
{
	int num=9;
	num += val;
	return num;
}