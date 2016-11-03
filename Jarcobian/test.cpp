#include "library_set.h"
#include "rhs_math.h"
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <algorithm>
FILE *out=stdout;

#include <iostream>
int main(){
	Matrix2f A,b;
	A<<1,1,1,1;
	b<<1,2,3,1;
	cout << A << endl;
	cout << b << endl;
	Matrix2f x = A.ldlt().solve(b);
	cout << x << endl;
	return 0;
}