#include <stdio.h>

typedef struct div
{
	int quotient;   // ¸ò
	int remainder;  // ³ª¸ÓÁö
} Div;

extern Div IntDiv(int num1, int num2);

int main(void)
{
	Div val=IntDiv(5, 2);
	printf("¸ò: %d \n", val.quotient);
	printf("³ª¸ÓÁö: %d \n", val.remainder);
	return 0; 
}