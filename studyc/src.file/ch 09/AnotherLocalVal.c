#include <stdio.h>

int main(void)
{
	int cnt;
	for(cnt=0; cnt<3; cnt++)
	{
		int num=0;
		num++;
		printf("%d번째 반복, 지역변수 num은 %d. \n", cnt+1, num);
	}
	
	if(cnt==3)
	{
		int num=7;
		num++;
		printf("if문 내에 존재하는 지역변수 num은 %d. \n", num);
	}
	return 0;
}
