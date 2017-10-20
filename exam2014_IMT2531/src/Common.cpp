//Author: Bendik Hillestad, 131333
//EXAM 2014

#include "Common.h"

using namespace std;

bool endsWith(const string &str, const string &end)
{
	if (end.size() > str.size()) return false;
	return equal(end.rbegin(), end.rend(), str.rbegin());
}