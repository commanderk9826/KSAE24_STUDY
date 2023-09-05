#include <iostream>
using namespace std;

class SoSimple
{
private:
	int num;
public:
	SoSimple(int n) : num(n)
	{ }
	SoSimple& AddNum(int n)
	{
		num+=n;
		return *this;
	}
	void SimpleFunc () 
	{
		cout<<"SimpleFunc: "<<num<<endl;
	}
	void SimpleFunc () const
	{
		cout<<"const SimpleFunc: "<<num<<endl;
	}
};

void YourFunc(const SoSimple &obj)
{
	obj.SimpleFunc();
}

int main(void)
{
	SoSimple obj1(2);
	const SoSimple obj2(7);

	obj1.SimpleFunc();
	obj2.SimpleFunc();

	YourFunc(obj1);
	YourFunc(obj2);
	return 0;
}
