#include <stdio.h>

int main(void)
{
	int result;
	int num1, num2;

	printf("정수 one: ");
	scanf("%d", &num1);	    // 첫 번째 정수 입력
	printf("정수 two: ");
	scanf("%d", &num2);	    // 두 번째 정수 입력
	
	result=num1+num2;
	printf("%d + %d = %d \n", num1, num2, result);
	return 0;
}
