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
	CHighLevelController(double dt);
	virtual ~CHighLevelController();

    void read(); //input: joint angle, joint velocity, imu (quaternian) 
    void write(); //output: torque
    void compute_controller(); 

    private:
    void QP_contact_force(); //


};

#endif