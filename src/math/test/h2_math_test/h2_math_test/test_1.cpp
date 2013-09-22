#include <iostream>
#include "..\..\..\h2_math.h"

using namespace std;

int main()
{
	cout << "h2_math test\n" << endl;

	int nErrors = 0;

	float roots[3];
	int sol = h2::solveThirdDegreeEquation(100.0f, 0.0f, 11.0f, 2.0f, roots);
	cout << "Solutions: " << sol << endl;

	cout << roots[0] << endl;
	cout << roots[1] << endl;
	cout << roots[2] << endl;

	cout << "\nTest finished. Errors: " << nErrors << endl;

	return 0;
}