#include <stdio.h>

int main(void)
{
	int num=-16;     // 11111111 11111111 11111111 11110000
	printf("2칸 오른쪽 이동의 결과: %d \n", num>>2);
	printf("3칸 오른쪽 이동의 결과: %d \n", num>>3);
	return 0;
}