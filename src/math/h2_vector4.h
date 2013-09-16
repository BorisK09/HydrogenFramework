


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Vector4f
	{
	public:

		static const unsigned int dimension = 4;

		union 
		{
			struct 
			{
				float x, y, z, w;
			};

			float v[4];
		};


		// Constructors 

		Vector4f() : x(0), y(0), z(0), w(0) {}

		Vector4f(const float x, const float y, const float z, const float w) : x(x), y(y), z(z), w(w) {}

		Vector4f(const float* arr) : x(arr[0]), y(arr[1]), z(arr[2]), w(arr[3]) {}

		Vector4f(Vector2f const &vec, const float z, const float w) : x(vec.x), y(vec.y), z(z), w(w) {}

		Vector4f(const float x, Vector2f const &vec, const float w) : x(x), y(vec.x), z(vec.y), w(w) {}

		Vector4f(const float x, const float y, Vector2f const &vec) : x(x), y(y), z(vec.x), w(vec.y) {}

		Vector4f(Vector2f const &vecA, Vector2f const &vecB) : x(vecA.x), y(vecA.y), z(vecB.x), w(vecB.y) {}

		Vector4f(Vector3f const &vec, const float w) : x(vec.x), y(vec.y), z(vec.z), w(w) {}

		Vector4f(const float x, Vector3f const &vec) : x(x), y(vec.x), z(vec.y), w(vec.z) {}

		template <class VecType>
		Vector4f(VecType const &vec) : x(vec.v[0]), y(vec.v[1]), z(vec.v[2]), w(vec.v[3]) {}


		// Copy

		Vector4f& operator = (const Vector4f &vec)
		{
			x = vec.x;
			y = vec.y;
			z = vec.z;
			w = vec.w;
			return *this;
		}


		// Unary operators

		inline Vector4f operator + () const { return *this; }

		inline Vector4f operator - () const { return Vector4f(-x, -y, -z, -w); }


		// Binary operators

		inline Vector4f operator + (const Vector4f& vec) const
		{
			return Vector4f(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
		}

		inline Vector4f operator - (const Vector4f& vec) const
		{
			return Vector4f(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
		}

		inline Vector4f operator * (float val) const
		{
			return Vector4f(x*val, y*val, z*val, w*val);
		}

		inline Vector4f operator * (const Vector4f& vec) const
		{
			return Vector4f(x*vec.x, y*vec.y, z*vec.z, z*vec.w);
		}

		friend Vector4f operator * (float val, const Vector4f& vec)
		{
			return Vector4f(val * vec.x, val * vec.y, val * vec.z, val * vec.w);
		}

		inline Vector4f operator / (float val) const
		{
			float invVal = 1.0f/val;
			return Vector4f(x*invVal, y*invVal, z*invVal, w*invVal);
		}


		// Assigment operators

		inline Vector4f& operator += (const Vector4f& vec)
		{
			x += vec.x;
			y += vec.y;
			z += vec.z;
			w += vec.w;
			return *this;
		}

		inline Vector4f& operator -= (const Vector4f& vec)
		{
			x -= vec.x;
			y -= vec.y;
			z -= vec.z;
			w -= vec.w;
			return *this;
		}

		inline Vector4f& operator *= (const Vector4f& vec)
		{
			x *= vec.x;
			y *= vec.y;
			z *= vec.z;
			w *= vec.w;
			return *this;
		}

		inline Vector4f& operator *= (float val)
		{
			x *= val;
			y *= val;
			z *= val;
			w *= val;
			return *this;
		}

		inline Vector4f& operator /= (float val)
		{
			float invVal = 1.0f/val;
			x *= invVal;
			y *= invVal;
			z *= invVal;
			w *= invVal;
			return *this;
		}


		// Methods

		inline float lenght()
		{
			return sqrt(x*x + y*y + z*z + w*w);
		}

		inline float sqlenght()
		{
			return x*x + y*y + z*z + w*w;
		}

		Vector4f normalize()
		{
			float lenght = sqrt(x*x + y*y + z*z + w*w);
			if (lenght != 0) 
			{
				float invLenght = 1.0f/lenght;
				return Vector4f(x*invLenght, y*invLenght, z*invLenght, w*invLenght);
			} else {
				return Vector4f(x, y, z, w);
			}
		}

		inline float dot(const Vector4f& vec)
		{
			return x*vec.x + y*vec.y + z*vec.z + w*vec.w;
		}
	};
}