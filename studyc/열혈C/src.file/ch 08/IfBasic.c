#include <stdio.h>

int main(void)
{
	int num;
	printf("정수 입력: ");
	scanf("%d", &num);
	
	if(num<0)     // num이 0보다 작으면 아래의 문장 실행
		printf("입력 값은 0보다 작다. \n");
	
	if(num>0)     // num이 0보다 크면 아래의 문장 실행
		printf("입력 값은 0보다 크다. \n");
	
	if(num==0)     // num이 0이면 아래의 문장 실행
		printf("입력 값은 0이다. \n");
	
	return 0;
}