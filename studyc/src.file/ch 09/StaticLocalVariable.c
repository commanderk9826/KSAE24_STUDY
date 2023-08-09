#include <stdio.h>

void SimpleFunc(void)
{
	static int num1=0;
	int num2=0;
	num1++, num2++;
	printf("static: %d, local: %d \n",num1, num2);
}

int main(void)
{
	int i;
	for(i=0; i<3; i++)
		SimpleFunc();
	return 0;
}