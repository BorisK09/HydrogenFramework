


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Matrix2x2f
	{
	public:

		static const unsigned int nRows = 2, nColumns = 2;

		union
		{
			struct
			{
				float _m00, _m01,
					  _m10, _m11;
			};

			float m[2][2];

			float mv[4];
		};


		// Constructors

		Matrix2x2f() : _m00(0), _m01(0),
					   _m10(0), _m11(0) {}

		Matrix2x2f(float in_m00, float in_m01,
				   float in_m10, float in_m11) : _m00(in_m00), _m01(in_m01),
												 _m10(in_m10), _m11(in_m11) {}

		template <class MatType>
		Matrix2x2f(MatType const &mat) : _m00(mat.m[0][0]), _m01(mat.m[0][1]),
										 _m10(mat.m[1][0]), _m11(mat.m[1][1]) {}


		// Copy

		inline Matrix2x2f& operator = (const Matrix2x2f &mat)
		{
			_m00 = mat._m00;
			_m01 = mat._m01;
			_m10 = mat._m10;
			_m11 = mat._m11;
			return *this;
		}


		// Unary operators

		inline Matrix2x2f operator + () const { return *this; }

		inline Matrix2x2f operator - () const 
		{
			return Matrix2x2f(-_m00, -_m01,
							  -_m10, -_m11);
		}


		// Binary operators

		inline Matrix2x2f operator + (const Matrix2x2f& mat) const
		{
			return Matrix2x2f(_m00 + mat._m00, _m01 + mat._m01,
							  _m10 + mat._m10, _m11 + mat._m11);
		}

		inline Matrix2x2f operator - (const Matrix2x2f& mat) const
		{
			return Matrix2x2f(_m00 - mat._m00, _m01 - mat._m01,
							  _m10 - mat._m10, _m11 - mat._m11);
		}

		inline Matrix2x2f operator * (float val) const
		{
			return Matrix2x2f(_m00 * val, _m01 * val,
							  _m10 * val, _m11 * val);
		}

		friend Matrix2x2f operator * (float val, const Matrix2x2f& mat)
		{
			return Matrix2x2f(mat._m00 * val, mat._m01 * val,
							  mat._m10 * val, mat._m11 * val);
		}

		inline Vector2f operator * (const Vector2f& vec) const
		{
			return Vector2f(_m00*vec.x + _m01*vec.y,
							_m10*vec.x + _m11*vec.y);
		}

		friend Vector2f operator * (const Vector2f& vec, const Matrix2x2f& mat)
		{
			return Vector2f(vec.x*mat._m00 + vec.y*mat._m10,
							vec.x*mat._m01 + vec.y*mat._m11);
		}

		inline Matrix2x2f operator * (const Matrix2x2f& mat) const
		{
			Matrix2x2f rMat;

			rMat._m00 = _m00*mat._m00 + _m01*mat._m10;
			rMat._m01 = _m00*mat._m01 + _m01*mat._m11;
			rMat._m10 = _m10*mat._m00 + _m11*mat._m10;
			rMat._m11 = _m10*mat._m01 + _m11*mat._m11;

			return rMat;
		}

		inline Matrix2x2f operator / (float val) const
		{
			float invVal = 1.0f/val;
			return Matrix2x2f(_m00 * invVal, _m01 * invVal,
							  _m10 * invVal, _m11 * invVal);
		}


		// Assigment operators

		inline Matrix2x2f& operator += (const Matrix2x2f& mat)
		{
			_m00 += mat._m00;
			_m01 += mat._m01;
			_m10 += mat._m10;
			_m11 += mat._m11;
			return *this;
		}

		inline Matrix2x2f& operator -= (const Matrix2x2f& mat)
		{
			_m00 -= mat._m00;
			_m01 -= mat._m01;
			_m10 -= mat._m10;
			_m11 -= mat._m11;
			return *this;
		}

		inline Matrix2x2f& operator *= (float val)
		{
			_m00 *= val;
			_m01 *= val;
			_m10 *= val;
			_m11 *= val;
			return *this;
		}

		inline Matrix2x2f& operator *= (const Matrix2x2f& mat)
		{
			_m00 = _m00*mat._m00 + _m01*mat._m10;
			_m01 = _m00*mat._m01 + _m01*mat._m11;
			_m10 = _m10*mat._m00 + _m11*mat._m10;
			_m11 = _m10*mat._m01 + _m11*mat._m11;
			return *this;
		}

		inline Matrix2x2f& operator /= (float val)
		{
			float invVal = 1.0f/val;
			_m00 *= invVal;
			_m01 *= invVal;
			_m10 *= invVal;
			_m11 *= invVal;
			return *this;
		}


		// Methods

		inline Matrix2x2f& setZero()
		{
			_m00 = 0;
			_m01 = 0;
			_m10 = 0;
			_m11 = 0;
			return *this;
		}

		inline Matrix2x2f& setIdentity()
		{
			_m00 = 1.0f;
			_m01 = 0;
			_m10 = 0;
			_m11 = 1.0f;
			return *this;
		}

		inline Matrix2x2f transpose() const
		{
			return Matrix2x2f(_m00, _m10,
							  _m01, _m11);
		}

		inline float determinant() const
		{
			return _m00*_m11 - _m01*_m10;
		}

		inline Matrix2x2f& setRow(unsigned int row, float valA, float valB)
		{
			m[row][0] = valA;
			m[row][1] = valB;
			return *this;
		}

		inline Matrix2x2f& setRow(unsigned int row, const Vector2f& vec)
		{
			m[row][0] = vec.x;
			m[row][1] = vec.y;
			return *this;
		}

		inline Matrix2x2f& setColumn(unsigned int column, float valA, float valB)
		{
			m[0][column] = valA;
			m[1][column] = valB;
			return *this;
		}

		inline Matrix2x2f& setColumn(unsigned int column, const Vector2f& vec)
		{
			m[0][column] = vec.x;
			m[1][column] = vec.y;
			return *this;
		}
	};
}