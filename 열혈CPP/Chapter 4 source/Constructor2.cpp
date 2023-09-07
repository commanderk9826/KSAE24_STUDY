#include <iostream>
using namespace std;

class SimpleClass
{
	int num1;
	int num2;

public:
	SimpleClass(int n1=0, int n2=0)
	{
		num1=n1;
		num2=n2;
	}

	void ShowData() const
	{
		cout<<num1<<' '<<num2<<endl;
	}
};

int main(void)
{
	SimpleClass sc1();    // 함수의 선언!

	SimpleClass mysc=sc1();
	mysc.ShowData();
	return 0;
}

SimpleClass sc1()
{
	SimpleClass sc(20, 30);
	return sc;
}