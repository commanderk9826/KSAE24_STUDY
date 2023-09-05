#include <iostream>
using namespace std;

template <typename T, int len>
class SimpleArray
{
private:
	T arr[len];
public:
	
	T& operator[] (int idx)
	{
		return arr[idx];
	}
	T& operator=(const T&ref)
	{
		for(int i=0; i<len; i++)
			arr[i]=ref.arr[i];
	}
};

int main(void)
{	
	SimpleArray<int, 5> i5arr1;
	for(int i=0; i<5; i++)
		i5arr1[i]=i*10;

	SimpleArray<int, 5> i5arr2;
	i5arr2=i5arr1;
	for(int i=0; i<5; i++)
		cout<<i5arr2[i]<<", ";
	cout<<endl;

	SimpleArray<int, 7> i7arr1;
	for(int i=0; i<7; i++)
		i7arr1[i]=i*10;

	SimpleArray<int, 7> i7arr2;
	i7arr2=i7arr1;
	for(int i=0; i<7; i++)
		cout<<i7arr2[i]<<", ";
	cout<<endl;
	return 0;
}
