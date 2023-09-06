#include <stdio.h>

int main(void)
{
	int num;
	
	for(num=1; num<100; num++)
	{
		if(num%3==0 || num%4==0)
			printf("3 또는 4의 배수: %d \n", num);
	}
	return 0;
}
