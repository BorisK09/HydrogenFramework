


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Quaternion
	{
	public:

		union
		{
			struct
			{
				float x, y, z, w;
			};

			float q[4];
		};


		// Constructors

		Quaternion() : x(0), y(0), z(0), w(0) {}

		Quaternion(float in_x, float in_y, float in_z, float in_w) : x(in_x), y(in_y), z(in_z), w(in_w) {}

		Quaternion(const Vector3f& vec, float scalar) : x(vec.x), y(vec.y), z(vec.z), w(scalar) {}

		Quaternion(const float* arr) : x(arr[0]), y(arr[1]), z(arr[2]), w(arr[3]) {}


		// Copy

		Quaternion& operator = (const Quaternion& quat)
		{
			x = quat.x;
			y = quat.y;
			z = quat.z;
			w = quat.w;
			return *this;
		}

		Quaternion& operator = (const Vector4f& vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;
			w = vec.w;
			return *this;
		}


		// Binary operators

		inline Quaternion operator + (const Quaternion& quat) const
		{
			return Quaternion(x + quat.x, y + quat.y, z + quat.z, w + quat.w);
		}

		inline Quaternion operator - (const Quaternion& quat) const
		{
			return Quaternion(x - quat.x, y - quat.y, z - quat.z, w - quat.w);
		}
		
		inline Quaternion operator * (float val) const
		{
			return Quaternion(x*val, y*val, z*val, w*val);
		}

		inline Quaternion operator * (const Quaternion& quat) const
		{
			//qq' = [ vxv' + wv' + w'v, ww' – v•v' ]
			//      vxv' — cross product, v•v' — dot product.

			Quaternion rQuat;

			rQuat.x = y*quat.z - z*quat.y + w*quat.x + quat.w*x;
			rQuat.y = z*quat.x - x*quat.z + w*quat.y + quat.w*y;
			rQuat.z = x*quat.y - y*quat.x + w*quat.z + quat.w*z;
			rQuat.w = w*quat.w - x*quat.x - y*quat.y - z*quat.z;

			return rQuat;
		}


		// Assigment operators

		inline Quaternion& operator += (const Quaternion& quat)
		{
			x += quat.x;
			y += quat.y;
			z += quat.z;
			w += quat.w;

			return *this;
		}

		inline Quaternion& operator -= (const Quaternion& quat)
		{
			x -= quat.x;
			y -= quat.y;
			z -= quat.z;
			w -= quat.w;

			return *this;
		}

		inline Quaternion& operator *= (float val)
		{
			x *= val;
			y *= val;
			z *= val;
			w *= val;

			return *this;
		}

		inline Quaternion& operator *= (const Quaternion& quat)
		{
			x = y*quat.z - z*quat.y + w*quat.x + quat.w*x;
			y = z*quat.x - x*quat.z + w*quat.y + quat.w*y;
			z = x*quat.y - y*quat.x + w*quat.z + quat.w*z;
			w = w*quat.w - x*quat.x - y*quat.y - z*quat.z;

			return *this;
		}

		inline Quaternion& operator /= (float val)
		{
			float invVal = 1.0f/val;
			x *= invVal;
			y *= invVal;
			z *= invVal;
			w *= invVal;

			return *this;
		}


		// Methods

		float norm() const
		{
			return x*x + y*y + z*z + w*w;
		}

		float lenght() const
		{
			return sqrt(x*x + y*y + z*z + w*w);
		}

		Quaternion conjugate() const
		{
			return Quaternion(-x, -y, -z, w);
		}

		Quaternion inverse() const
		{
			float norm = x*x + y*y + z*z + w*w;
			return Quaternion(-x/norm, -y/norm, -z/norm, w/norm);
		}

		float dot(const Quaternion& quat) // Inner product of two quaternions
		{
			return x*quat.x + y*quat.y + y*quat.y + y*quat.y;
		}
	};
}