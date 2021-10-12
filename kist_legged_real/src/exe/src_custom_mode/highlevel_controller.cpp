#ifndef __HIGHLEVELCONTROLLER_H
#define __HIGHLEVELCONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "include_custom_mode/custommath.h"
#include "include_custom_mode/robotmodel.h"


using namespace std;
using namespace Eigen;

class CHighLevelController
{
    public:
	CTaskController(int jdof, double dt, const double touch_probe_position_rad[]);
	virtual ~CTaskController();


};

#endif