


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Vector2f
	{
	public:

		static const unsigned int dimension = 2;

		union 
		{
			struct 
			{
				float x, y;
			};

			float v[2];
		};


		// Constructors

		Vector2f() : x(0), y(0) {}

		Vector2f(const float x, const float y) : x(x), y(y) {}

		Vector2f(const float* arr) : x(arr[0]), y(arr[1]) {}

		template <class VecType>
		Vector2f(VecType const &vec) : x(vec.v[0]), y(vec.v[1]) {}


		// Copy

		inline Vector2f& operator = (const Vector2f &vec)
		{
			x = vec.x;
			y = vec.y;
			return *this;
		}


		// Unary operators

		inline Vector2f operator + () const { return *this; }

		inline Vector2f operator - () const { return Vector2f(-x, -y); }


		// Binary operators

		inline Vector2f operator + (const Vector2f& vec) const
		{
			return Vector2f(x + vec.x, y + vec.y);
		}

		inline Vector2f operator - (const Vector2f& vec) const
		{
			return Vector2f(x - vec.x, y - vec.y);
		}

		inline Vector2f operator * (float val) const
		{
			return Vector2f(x*val, y*val);
		}

		inline Vector2f operator * (const Vector2f& vec) const
		{
			return Vector2f(x*vec.x, y*vec.y);
		}

		friend Vector2f operator * (float val, const Vector2f& vec)
		{
			return Vector2f(val * vec.x, val * vec.y);
		}

		inline Vector2f operator / (float val) const
		{
			float invVal = 1.0f/val;
			return Vector2f(x*invVal, y*invVal);
		}


		// Assigment operators

		inline Vector2f& operator += (const Vector2f& vec)
		{
			x += vec.x;
			y += vec.y;
			return *this;
		}

		inline Vector2f& operator -= (const Vector2f& vec)
		{
			x -= vec.x;
			y -= vec.y;
			return *this;
		}

		inline Vector2f& operator *= (const Vector2f& vec)
		{
			x *= vec.x;
			y *= vec.y;
			return *this;
		}

		inline Vector2f& operator *= (float val)
		{
			x *= val;
			y *= val;
			return *this;
		}

		inline Vector2f& operator /= (float val)
		{
			float invVal = 1.0f/val;
			x *= invVal;
			y *= invVal;
			return *this;
		}


		// Methods

		inline float lenght()
		{
			return sqrt(x*x + y*y);
		}

		inline float sqlenght()
		{
			return x*x + y*y;
		}

		Vector2f normalize()
		{
			float lenght = sqrt(x*x + y*y);
			float invLenght;
			if (lenght != 0) 
			{
				invLenght = 1.0f/lenght;
				return Vector2f(x*invLenght, y*invLenght);
			} else {
				return Vector2f(x, y);
			}
		}

		inline float dot(const Vector2f& vec) const
		{
			return x*vec.x + y*vec.y;
		}

	};
}