#include <iostream>
using namespace std;

class Data
{
private:
	int data;
public:
	Data(int num): data(num)
	{  }
	void ShowData()
	{
		cout<<"Data: "<<data<<endl;
	}
	void Add(int num)
	{
		data+=num;
	}
};

int main(void)
{
	Data obj(15);
	obj.Add(17);
	obj.ShowData();
	return 0;
}