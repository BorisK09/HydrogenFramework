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



#define H2_PI             3.14159265f
#define H2_2PI            6.28318530f
#define H2_THIRD          0.33333333f
#define H2_EQN_EPS        1e-9
#define	H2_IS_ZERO(x)	 ((x) > -H2_EQN_EPS && (x) < H2_EQN_EPS)
#define	H2_CUBIC_ROOT(x) ((x) > 0.0f ? pow((float)(x), 1.0f/3.0f) : \
                         ((x) < 0.0f ? -pow((float)-(x), 1.0f/3.0f) : 0.0f))




namespace h2
{
	template <class T>
	T abs(const T& val)
	{
		return val < 0 ? -val : val;
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
		return inVecA.dot(inVecB);
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


	/*template <class T>
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
	}*/


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
			float D = b*b - 4.0f*a*c;

			if (D > 0)   
			{
				outResult[0] = (-b + sqrt(D))/(2.0f*a);
				outResult[1] = (-b - sqrt(D))/(2.0f*a);
				return 2;	

			} else if (D == 0)
			{
				outResult[0] = -b/(2.0f*a);
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
	int solveThirdDegreeEquation(float a, float b, float c, float d, float* outResult)
	{
		if (a == 0)
		{
			return h2::solveSecondDegreeEquation(b, c, d, outResult);

		} else 
		{
			// Cardano's method

			// convert to normal form y^3 + Ay^2 + By + C = 0 by dividing to 'a'

			float A, B, C;

			A = b/a;
			B = c/a;
			C = d/a;

			float A2 = A*A;

			float offset = H2_THIRD*A;

			float p, q;

			p = H2_THIRD*(-H2_THIRD * A2 + B);
			q = 0.5f*(2.0f/27*A*A2 - H2_THIRD*A*B + C);

			float D = q*q + p*p*p;

			if (H2_IS_ZERO(D))
			{
				if (H2_IS_ZERO(q)) // one triple solution
				{
					outResult[0] = -offset;
					return 1;

				} else //one single and one double solution
				{	
					float u = H2_CUBIC_ROOT(-q);
					outResult[0] = 2.0f*u - offset;
					outResult[1] = -u - offset;
					return 2;
				}
			} else if (D < 0) // three real solutions
			{
				float phi = H2_THIRD*acos(-q/sqrt(-p*p*p));
				float t = 2.0f * sqrt(-p);

				outResult[0] =  t*cos(phi) - offset;
				outResult[1] = -t*cos(phi + H2_PI*H2_THIRD) - offset;
				outResult[2] = -t*cos(phi - H2_PI*H2_THIRD) - offset;
				return 3;

			} else // one real solution
			{
				float sqrtD = sqrt(D);

				float u =  H2_CUBIC_ROOT(sqrtD - q);
				float v = -H2_CUBIC_ROOT(sqrtD + q);

				outResult[0] = u + v - offset;
				return 1;
			}
		}

	}


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