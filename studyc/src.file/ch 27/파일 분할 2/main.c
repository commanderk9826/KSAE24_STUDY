#include <stdio.h>

extern void AddCnt(void);
extern void MinCnt(void);
extern int GetCnt(void);

int main(void)
{
	AddCnt();
	AddCnt();
	printf("현재 참여자: %d \n", GetCnt());
	MinCnt();
	printf("현재 참여자: %d \n", GetCnt());
	return 0;
}