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



#define H2_PI    3.14159265f
#define H2_2PI   6.28318530f



namespace h2
{
	template <class T>
	T abs(const T& val)
	{
		return a < 0 ? -a : a;
	}

	
	template <class T>
	void abs(T* arr, unsigned int nElements)
	{
		for (unsigned int i = 0; i < nElements; i++)
		{
			arr[i] = (arr[i] < 0) ? -arr[i] : arr[i];
		}
	}


	template <class T>
	T ceil(const T& val, const T& min, const T& max)
	{
		if (val < min) 
		{
			return min;
		}

		if (val > max) 
		{
			return max;
		}

		return val;
	}


	h2::Vector3f cross(h2::Vector3f inVecA, h2::Vector3f inVecB)
	{
		return h2::Vector3f(inVecA.y * inVecB.z - inVecA.z * inVecB.y,
							inVecA.z * inVecB.x - inVecA.x * inVecB.z,
							inVecA.x * inVecB.y - inVecA.y * inVecB.x);
	}


	float degToRad(float deg)
	{
		return deg*H2_PI/180.0f;
	}


	template <class T>
	float determinant(T& mat)
	{
		return mat.determinant();
	}


	float distanance(const h2::Vector2f& pointA, const h2::Vector2f& pointB)
	{
		return sqrt(pointA.x*pointB.x + pointA.y*pointB.y);
	}


	float distanance(const h2::Vector3f& pointA, const h2::Vector3f& pointB)
	{
		return sqrt(pointA.x*pointB.x + pointA.y*pointB.y + pointA.z*pointB.z);
	}


	float distanance(const h2::Vector4f& pointA, const h2::Vector4f& pointB)
	{
		return sqrt(pointA.x*pointB.x + pointA.y*pointB.y + pointA.z*pointB.z + pointA.w*pointB.w);
	}


	/*float distanance(const h2::VectorNf& pointA, const h2::VectorNf& pointB)
	{
		return 0;
	}*/


	template <class T>
	float dot(const T& inVecA, const T& inVecB)
	{
		return inVecA.dot(inVecB);s
	}


	template <class T>
	T lerp(const T start, const T end, float t)
	{
		return start+(end-start)*t;
	}


	template <class T>
	T normalize(T inVec)
	{
		return inVec.normalize();
	}


	float radToDeg(float rad)
	{
		return rad*180.0f/H2_PI;
	}


	template <class T>
	T min(const T& a, const T& b)
	{
		return a <= b ? a : b;
	}


	template <class T>
	T max(const T& a, const T& b)
	{
		return a >= b ? a : b;
	}


	template <class T>
	T min(const T& a, const T& b, const T& c)
	{
		if (a <= b)
		{
			return a <= c ? a : c;
		} else {
			return b >= c ? b : c;
		}
	}


	template <class T>
	T max(const T& a, const T& b, const T& c)
	{
		if (a >= b)
		{
			return a >= c ? a : c;
		} else {
			return b >= c ? b : c;
		}
	}


	template <class T>
	T min(const T* arr, unsigned int nElements)
	{
		T curr_min = arr[0];

		for (unsigned int i = 1; i < nElements; i++)
		{
			if (arr[i] < curr_min)
			{
				curr_min = arr[i];
			}
		}
		return curr_min;
	}


	template <class T>
	T max(const T* arr, unsigned int nElements)
	{
		T curr_max = arr[0];

		for (unsigned int i = 1; i < nElements-1; i++)
		{
			if (arr[i] > curr_max)
			{
				curr_max = arr[i];
			}
		}
		return curr_max;
	}


	float slerp(float start, float end, float t)
	{
		float omega = acos(start*end);
		return sin((1.0f - t)*omega)*start/sin(omega) + sin(t*omega)*end/sin(omega);
	}


	// Slerp for Quaternions
	// http://en.wikipedia.org/wiki/Slerp
	// https://theory.org/software/qfa/writeup/node12.html
	// http://www.sonycsl.co.jp/person/nielsen/visualcomputing/programs/slerp.cpp
	/*Quaternion slerp(float start, float end, float t)
	{
		return 0;
	}
	*/


	template <class T>
	T slerp(const T start, const T end, float t)
	{
		float omega = acos(dot(start, end));
		T result = sin((1.0f - t)*omega)*start/sin(omega) + sin(t*omega)*end/sin(omega);
		return result;
	}


	// Function finds solution of decond degree equation (for x) 
	// ax^2 + bx + c = 0 and 
	// returns number of solutions.
	// Solutions returns via '*outResult' parameter.
	// Complex solutions are not supported.
	int solveSecondDegreeEquation(float a, float b, float c, float* outResult)
	{
		if (a == 0) // if 'a' is zero then 1st degree equation
		{
			if (b != 0)
			{
				outResult[0] = -c/b;
				return 1;

			} else // if 'b' is zero - no solutions
			{
				return 0;
			}

		} else
		{
			float D = b*b - 4*a*c;

			if (D > 0)   
			{
				outResult[0] = (-b + sqrt(D))/(2*a);
				outResult[1] = (-b - sqrt(D))/(2*a);
				return 2;	

			} else if (D == 0)
			{
				outResult[0] = -b/(2*a);
				return 1;

			} else // 'D' is negative - no solutions
			{
				return 0;
			}
		}
	}


	// Function finds solution of third degree equation (for x) 
	// ax^3 + bx^2 + cx + d = 0 and 
	// returns number of solutions.
	// Solutions returns via '*outResult' parameter.
	// Complex solutions are not supported.
	/*int solveThirdDegreeEquation(float a, float b, float c, float d, float* outResult)
	{
		return 0;
	}*/


	template <class T>
	T transpose(T& mat)
	{
		return mat.transpose();
	}

} // end of namespace 'h2'


int h2_math_test()
{
	return 0;
}