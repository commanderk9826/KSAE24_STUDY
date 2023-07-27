#include <stdio.h>

int main(void)
{
	double rad;
	double area;
	printf("원의 반지름 입력: ");
	scanf("%lf", &rad);	
	
	area = rad*rad*3.1415;
	printf("원의 넓이: %f \n", area);
	return 0;
}