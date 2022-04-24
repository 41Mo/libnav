#ifndef Pose_h__
#define Pose_h__

#include "math.hpp"

class Nav_solution
{
private:
    /*
        Orientation
    */

    /*
        Object rotation angles in body frame.
        0 element - roll;
        1 element - pitch;
        2 element - yaw;
    */
    matrix::Eulerf* rot_ptr;
    

    /*
        Navigation
    */

    /*
        Object velocity in ENU frame.
        0 element - V_e; east component
        1 element - V_n; north component
        2 element - V_up; vertical component
    */
   matrix::Vector3f* vel_ptr;

    /*
        Object coordinates in ENU frame.
        0 element - latitude;
        1 element - longtitude;
    */
   matrix::Vector2f* coord_ptr;

public:
	/*
		Get velocity vector component by vos in vector.
	*/
	const float &vel(int num);

	/*
		Get velocity vector.
	*/
    const matrix::Vector3f& vel();

    /*
        Fill vec_in with vel components.
    */
    void vel(float vec_in[3]);

	/*
		Get rotation angle vector component by pos in vector.
	*/
	const float &rot(int num);

	/*
		Get rotation vector.
	*/
    const matrix::Eulerf& rot();

    /*
        Fill vec_in with rot components.
    */
    void rot(float vec_in[3]);

	/*
		Get pos vector component by pos in vecotr.
	*/
    const float &pos(int num);

    /*
        Fill vec_in with pos components.
    */
    void pos(float vec_in[2]);

	/*
		Get position vector.
	*/
    const matrix::Vector2f& pos();


protected:

    friend class Nav;

    Nav_solution() { }
    void setup(matrix::Vector3f *vel, matrix::Eulerf *rpy, matrix::Vector2f *coord);

};
#endif // Pose_h__