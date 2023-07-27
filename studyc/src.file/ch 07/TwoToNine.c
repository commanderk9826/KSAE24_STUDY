#include <stdio.h>

int main(void)
{
	int cur=2;
	int is=0;

	while(cur<10)    // 2단부터 9단까지 반복
	{
		is=1;    // 새로운 단의 시작을 위해서
		while(is<10)    // 각 단의 1부터 9의 곱을 표현
		{
			printf("%d×%d＝%d \n", cur, is, cur*is);
			is++;
		}
		cur++;    // 다음 단으로 넘어가기 위한 증가
		printf("\n");
	}
	return 0;
}
