


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Vector3f
	{
	public:

		static const unsigned int dimension = 3;

		union 
		{
			struct 
			{
				float x, y, z;
			};

			float v[3];
		};


		// Constructors 

		Vector3f() : x(0), y(0), z(0) {}

		Vector3f(const float x, const float y, const float z) : x(x), y(y), z(z) {}

		Vector3f(const float* arr) : x(arr[0]), y(arr[1]), z(arr[2]) {}

		Vector3f(Vector2f const &vec, const float z) : x(vec.x), y(vec.y), z(z) {}

		Vector3f(const float x, Vector2f const &vec) : x(x), y(vec.x), z(vec.y) {}

		template <class VecType>
		Vector3f(VecType const &vec) : x(vec.v[0]), y(vec.v[1]), z(vec.v[2]) {}


		// Copy

		Vector3f& operator = (const Vector3f &vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;
			return *this;
		}


		// Unary operators

		inline Vector3f operator + () const { return *this; }

		inline Vector3f operator - () const { return Vector3f(-x, -y, -z); }


		// Binary operators

		inline Vector3f operator + (const Vector3f& vec) const
		{
			return Vector3f(x + vec.x, y + vec.y, z + vec.z);
		}

		inline Vector3f operator - (const Vector3f& vec) const
		{
			return Vector3f(x - vec.x, y - vec.y, z - vec.z);
		}

		inline Vector3f operator * (float val) const
		{
			return Vector3f(x*val, y*val, z*val);
		}

		inline Vector3f operator * (const Vector3f& vec) const
		{
			return Vector3f(x*vec.x, y*vec.y, z*vec.z);
		}

		friend Vector3f operator * (float val, const Vector3f& vec)
		{
			return Vector3f(val * vec.x, val * vec.y, val * vec.z);
		}

		inline Vector3f operator / (float val) const
		{
			float invVal = 1.0f/val;
			return Vector3f(x*invVal, y*invVal, z*invVal);
		}


		// Assigment operators

		inline Vector3f& operator += (const Vector3f& vec)
		{
			x += vec.x;
			y += vec.y;
			z += vec.z;
			return *this;
		}

		inline Vector3f& operator -= (const Vector3f& vec)
		{
			x -= vec.x;
			y -= vec.y;
			z -= vec.z;
			return *this;
		}

		inline Vector3f& operator *= (const Vector3f& vec)
		{
			x *= vec.x;
			y *= vec.y;
			z *= vec.z;
			return *this;
		}

		inline Vector3f& operator *= (float val)
		{
			x *= val;
			y *= val;
			z *= val;
			return *this;
		}

		inline Vector3f& operator /= (float val)
		{
			float invVal = 1.0f/val;
			x *= invVal;
			y *= invVal;
			z *= invVal;
			return *this;
		}


		// Methods

		inline float lenght()
		{
			return sqrt(x*x + y*y + z*z);
		}

		inline float sqlenght()
		{
			return x*x + y*y + z*z;
		}

		Vector3f normalize()
		{
			float lenght = sqrt(x*x + y*y + z*z);
			float invLenght;
			if (lenght != 0) 
			{
				invLenght = 1.0f/lenght;
				return Vector3f(x*invLenght, y*invLenght, z*invLenght);
			} else {
				return Vector3f(x, y, z);
			}
		}

		inline float dot(const Vector3f& vec)
		{
			return x*vec.x + y*vec.y + z*vec.z;
		}
	};
}