#include <stdio.h>
int NumberCompare(int num1, int num2);

int main(void)
{
	printf("3과 4중에서 큰 수는 %d 이다. \n", NumberCompare(3, 4));
	printf("7과 2중에서 큰 수는 %d 이다. \n", NumberCompare(7, 2));
	return 0;
}

int NumberCompare(int num1, int num2)
{
	if(num1>num2)
		return num1;
	else
		return num2;
}