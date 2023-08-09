#include <stdio.h>

int main(void)
{
	int total=0;
	int i, num;
	printf("0ºÎÅÍ num±îÁöÀÇ µ¡¼À, numÀº? ");
	scanf("%d", &num);
	
	for(i=0; i<num+1; i++)
		total+=i;
	
	printf("0ºÎÅÍ %d±îÁö µ¡¼À°á°ú: %d \n", num, total);
	return 0;
}
