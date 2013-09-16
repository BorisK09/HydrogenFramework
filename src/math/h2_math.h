#include <cmath>

#include "h2_vector2.h"
#include "h2_vector3.h"
#include "h2_vector4.h"
#include "h2_vectorn.h"

#include "h2_matrix2x2.h"
#include "h2_matrix3x3.h"
#include "h2_matrix4x4.h"
#include "h2_matrixmxn.h"

#include "h2_quaternion.h"





#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union




namespace h2
{


	Vector3f cross(Vector3f inVecA, Vector3f inVecB)
	{
		Vector3f outVec;

		outVec.x = inVecA.y * inVecB.z - inVecA.z * inVecB.y;
		outVec.y = inVecA.z * inVecB.x - inVecA.x * inVecB.z;
		outVec.z = inVecA.x * inVecB.y - inVecA.y * inVecB.x;

		return outVec;
	}


	template <class T>
	float dot(T inVecA, T inVecB)
	{
		return inVecA.dot(inVecB);
	}


	template <class T>
	T normalize(T inVec)
	{
		return inVec.normalize();
	}


	template <class T>
	T transpose(T& mat)
	{
		return mat.transpose();
	}


	template <class T>
	float determinant(T& mat)
	{
		return mat.determinant();
	}

} // end of namespace 'h2'


int h2_math_test()
{
	return 0;
}