#include <iostream>
using namespace std;

class BaseOne
{
public:
	void SimpleFuncOne() { cout<<"BaseOne"<<endl; }
};

class BaseTwo
{
public:
	void SimpleFuncTwo() { cout<<"BaseTwo"<<endl; }
};

class MultiDrived : public BaseOne, protected BaseTwo
{
public:
	void ComplexFunc()
	{
		SimpleFuncOne();
		SimpleFuncTwo();
	}
};

int main(void)
{
	MultiDrived mdr;
	mdr.ComplexFunc();
	return 0;
}