#include <iostream>
using namespace std;

int main(void)
{
	int num1, num2;
	cout<<"µÎ °³ÀÇ ¼ıÀÚ ÀÔ·Â: ";
	cin>>num1>>num2;

	cout<<"³ª´°¼ÀÀÇ ¸ò: "<< num1/num2 <<endl;
	cout<<"³ª´°¼ÀÀÇ ³ª¸ÓÁö: "<< num1%num2 <<endl;	
	return 0;
}