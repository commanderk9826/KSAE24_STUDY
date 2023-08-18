#include <stdio.h>

int main(void)
{
	int num1=100, num2=100;
	int * pnum;

	pnum=&num1;    // 포인터 pnum이 num1을 가리킴
	(*pnum)+=30;     // num1+=30과 동일

	pnum=&num2;     // 포인터 pnum이 num2를 가리킴
	(*pnum)-=30;     // num2-=30과 동일
	
	printf("num1:%d, num2:%d \n", num1, num2);
	return 0;
}
/*실행결과
num1:130, num2:70
*/