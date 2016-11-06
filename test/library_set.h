#pragma once
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <string>
#include <math.h>
#include <fstream>

using namespace std;
using namespace Eigen;

int drawType=1;
float PI = acos(-1.0);
double dt=0.0, alpha=0.02;

inline string trim(const string &s)
{
   auto wsfront=find_if_not(s.begin(),s.end(),[](int c){return isspace(c);});
   auto wsback=find_if_not(s.rbegin(),s.rend(),[](int c){return isspace(c);}).base();
   return (wsback<=wsfront ? string() : string(wsfront,wsback));
}
