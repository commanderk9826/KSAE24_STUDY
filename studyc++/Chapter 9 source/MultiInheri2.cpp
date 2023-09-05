#include <iostream>
using namespace std;

class BaseOne
{
public:
	void SimpleFunc() { cout<<"BaseOne"<<endl; }
};

class BaseTwo
{
public:
	void SimpleFunc() { cout<<"BaseTwo"<<endl; }
};

class MultiDrived : public BaseOne, protected BaseTwo
{
public:
	void ComplexFunc()
	{
		BaseOne::SimpleFunc();
		BaseTwo::SimpleFunc();
	}
};

int main(void)
{
	MultiDrived mdr;
	mdr.ComplexFunc();
	return 0;
}