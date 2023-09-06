#include <stdio.h>

int main(void)
{
	int arr[3]={0, 1, 2};
	printf("배열의 이름: %p \n", arr);
	printf("첫 번째 요소: %p \n", &arr[0]);
	printf("두 번째 요소: %p \n", &arr[1]);
	printf("세 번째 요소: %p \n", &arr[2]);
	return 0;
}
/*실행결과
배열의 이름 : 0012FF50
첫 번째 요소 : 0012FFF50
두 번째 요소 : 0012FF54
세 번째 요소 : 0012FF58
*/
