#ifndef PARTITION_H
#define PARTITION_H

#include <iostream>
#include <set>
#include <vector>
#include <string>

struct Edge
{
	int v1, v2;
	double energy;
	Edge(int a, int b, double c) : v1(a), v2(b), energy(c) {}
	bool operator<(const Edge& rhs) const
	{
		return energy < rhs.energy;
	}
};

class Partition
{
public:
	Partition() {}
	~Partition() {}

	bool readPLY(const std::string filename);

private:
	std::set<Edge> heap_;
};



#endif // !PARTITION_H
