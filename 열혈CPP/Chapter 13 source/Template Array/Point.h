#ifndef __POINT_H_
#define __POINT_H_

#include <iostream>
using namespace std;

class Point
{
private:
	int xpos, ypos;
public:
	Point(int x=0, int y=0);
	friend ostream& operator<<(ostream& os, const Point& pos);
};

#endif