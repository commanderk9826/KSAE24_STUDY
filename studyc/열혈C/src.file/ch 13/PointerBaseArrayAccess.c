#include <stdio.h>

int main(void)
{
	int arr[3]={11, 22, 33};
	int * ptr=arr;     // int * ptr=&arr[0]; 과 같은 문장

	printf("%d %d %d \n", *ptr, *(ptr+1), *(ptr+2));

	printf("%d ", *ptr);  ptr++; //printf 함수호출 후, ptr++ 실행
	printf("%d ", *ptr);  ptr++;
	printf("%d ", *ptr);  ptr--; //printf 함수호출 후, ptr-- 실행
	printf("%d ", *ptr);  ptr--;
	printf("%d ", *ptr);  printf("\n");	
	return 0;
}
/*실행결과
11 22 33
11 22 33 22 11
*/