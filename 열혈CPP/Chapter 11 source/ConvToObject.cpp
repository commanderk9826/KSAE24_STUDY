#include <iostream>
using namespace std;

class Number
{
private:
	int num;
public:
	Number(int n=0) : num(n)
	{
		cout<<"Number(int n=0)"<<endl;
	}
	Number& operator=(const Number &ref)
	{
		cout<<"operator=()"<<endl;
		num=ref.num;
		return *this;
	}
	void ShowNumber() { cout<<num<<endl; }
};

int main(void)
{
	Number num;
	num=30;
	num.ShowNumber();
	return 0;
}
