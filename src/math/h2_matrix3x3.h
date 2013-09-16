


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Matrix3x3f
	{
	public:

		static const unsigned int nRows = 3, nColumns = 3;

		union
		{
			struct
			{
				float _m00, _m01, _m02,
					  _m10, _m11, _m12,
					  _m20, _m21, _m22;
			};

			float m[3][3];

			float mv[9];
		};


		// Constructors

		Matrix3x3f() : _m00(0), _m01(0), _m02(0),
					   _m10(0), _m11(0), _m12(0),
					   _m20(0), _m21(0), _m22(0) {}

		Matrix3x3f(float in_m00, float in_m01, float in_m02,
				   float in_m10, float in_m11, float in_m12,
				   float in_m20, float in_m21, float in_m22) : _m00(in_m00), _m01(in_m01), _m02(in_m02),
															   _m10(in_m10), _m11(in_m11), _m12(in_m12),
															   _m20(in_m20), _m21(in_m21), _m22(in_m22) {}

		template <class MatType>
		Matrix3x3f(MatType const &mat) : _m00(mat.m[0][0]), _m01(mat.m[0][1]), _m02(mat.m[0][2]),
										 _m10(mat.m[1][0]), _m11(mat.m[1][1]), _m12(mat.m[1][2]),
										 _m20(mat.m[2][0]), _m21(mat.m[2][1]), _m22(mat.m[2][2]) {}


		// Copy

		inline Matrix3x3f& operator = (const Matrix3x3f &mat)
		{
			_m00 = mat._m00;  _m01 = mat._m01;  _m02 = mat._m02;
			_m10 = mat._m10;  _m11 = mat._m11;  _m12 = mat._m12;
			_m20 = mat._m20;  _m21 = mat._m21;  _m22 = mat._m22;
			return *this;
		}

		inline Matrix3x3f& operator = (const Matrix2x2f &mat)
		{
			_m00 = mat._m00;  _m01 = mat._m01;
			_m10 = mat._m10;  _m11 = mat._m11;
			return *this;
		}


		// Unary operators

		inline Matrix3x3f operator + () const { return *this; }

		inline Matrix3x3f operator - () const 
		{
			return Matrix3x3f(-_m00, -_m01, -_m02,
							  -_m10, -_m11, -_m12,
							  -_m20, -_m21, -_m22);
		}


		// Binary operators

		inline Matrix3x3f operator + (const Matrix3x3f& mat) const
		{
			return Matrix3x3f(_m00 + mat._m00,  _m01 + mat._m01,  _m02 + mat._m02,
							  _m10 + mat._m10,  _m11 + mat._m11,  _m12 + mat._m12,
							  _m20 + mat._m20,  _m21 + mat._m21,  _m22 + mat._m22);
		}

		inline Matrix3x3f operator - (const Matrix3x3f& mat) const
		{
			return Matrix3x3f(_m00 - mat._m00,  _m01 - mat._m01,  _m02 - mat._m02,
							  _m10 - mat._m10,  _m11 - mat._m11,  _m12 - mat._m12,
							  _m20 - mat._m20,  _m21 - mat._m21,  _m22 - mat._m22);
		}

		inline Matrix3x3f operator * (float val) const
		{
			return Matrix3x3f(_m00*val, _m01*val, _m02*val,
							  _m10*val, _m11*val, _m12*val,
							  _m20*val, _m21*val, _m22*val);
		}

		friend Matrix3x3f operator * (float val, const Matrix3x3f& mat)
		{
			return Matrix3x3f(mat._m00*val, mat._m01*val, mat._m02*val,
							  mat._m10*val, mat._m11*val, mat._m12*val,
							  mat._m20*val, mat._m21*val, mat._m22*val);
		}

		inline Vector3f operator * (const Vector3f& vec) const
		{
			return Vector3f(_m00*vec.x + _m01*vec.y + _m02*vec.z,
							_m10*vec.x + _m11*vec.y + _m12*vec.z,
							_m20*vec.x + _m21*vec.y + _m22*vec.z);
		}

		friend Vector3f operator * (const Vector3f& vec, const Matrix3x3f& mat)
		{
			return Vector3f(vec.x*mat._m00 + vec.y*mat._m10 + vec.z*mat._m20,
							vec.x*mat._m01 + vec.y*mat._m11 + vec.z*mat._m21,
							vec.x*mat._m02 + vec.y*mat._m12 + vec.z*mat._m22);
		}

		inline Matrix3x3f operator * (const Matrix3x3f& mat) const
		{
			Matrix3x3f rMat;

			rMat._m00 = _m00*mat._m00 + _m01*mat._m10 + _m02*mat._m20;
			rMat._m01 = _m00*mat._m01 + _m01*mat._m11 + _m02*mat._m21;
			rMat._m02 = _m00*mat._m02 + _m01*mat._m12 + _m02*mat._m22;
			rMat._m10 = _m10*mat._m00 + _m11*mat._m10 + _m12*mat._m20;
			rMat._m11 = _m10*mat._m01 + _m11*mat._m11 + _m12*mat._m21;
			rMat._m12 = _m10*mat._m02 + _m11*mat._m12 + _m12*mat._m22;
			rMat._m20 = _m20*mat._m00 + _m21*mat._m10 + _m22*mat._m20;
			rMat._m21 = _m20*mat._m01 + _m21*mat._m11 + _m22*mat._m21;
			rMat._m22 = _m20*mat._m02 + _m21*mat._m12 + _m22*mat._m22;

			return rMat;
		}

		inline Matrix3x3f operator / (float val) const
		{
			float invVal = 1.0f/val;
			return Matrix3x3f(_m00*invVal, _m01*invVal, _m02*invVal,
							  _m10*invVal, _m11*invVal, _m12*invVal,
							  _m20*invVal, _m21*invVal, _m22*invVal);
		}


		// Assigment operators

		inline Matrix3x3f& operator += (const Matrix3x3f& mat)
		{
			_m00 += mat._m00;  _m01 += mat._m01;  _m02 += mat._m02;
			_m10 += mat._m10;  _m11 += mat._m11;  _m12 += mat._m12;
			_m20 += mat._m20;  _m21 += mat._m21;  _m22 += mat._m22;
			return *this;
		}

		inline Matrix3x3f& operator -= (const Matrix3x3f& mat)
		{
			_m00 -= mat._m00;  _m01 -= mat._m01;  _m02 -= mat._m02;
			_m10 -= mat._m10;  _m11 -= mat._m11;  _m12 -= mat._m12;
			_m20 -= mat._m20;  _m21 -= mat._m21;  _m22 -= mat._m22;
			return *this;
		}

		inline Matrix3x3f& operator *= (float val)
		{
			_m00 *= val;  _m01 *= val;  _m02 *= val;
			_m10 *= val;  _m11 *= val;  _m12 *= val;
			_m20 *= val;  _m21 *= val;  _m22 *= val;
			return *this;
		}

		inline Matrix3x3f& operator *= (const Matrix3x3f& mat)
		{
			_m00 = _m00*mat._m00 + _m01*mat._m10 + _m02*mat._m20;
			_m01 = _m00*mat._m01 + _m01*mat._m11 + _m02*mat._m21;
			_m02 = _m00*mat._m02 + _m01*mat._m12 + _m02*mat._m22;
			_m10 = _m10*mat._m00 + _m11*mat._m10 + _m12*mat._m20;
			_m11 = _m10*mat._m01 + _m11*mat._m11 + _m12*mat._m21;
			_m12 = _m10*mat._m02 + _m11*mat._m12 + _m12*mat._m22;
			_m20 = _m20*mat._m00 + _m21*mat._m10 + _m22*mat._m20;
			_m21 = _m20*mat._m01 + _m21*mat._m11 + _m22*mat._m21;
			_m22 = _m20*mat._m02 + _m21*mat._m12 + _m22*mat._m22;
			return *this;
		}

		inline Matrix3x3f& operator /= (float val)
		{
			float invVal = 1.0f/val;
			_m00 *= invVal;  _m01 *= invVal;  _m02 *= invVal;
			_m10 *= invVal;  _m11 *= invVal;  _m12 *= invVal;
			_m20 *= invVal;  _m21 *= invVal;  _m22 *= invVal;
			return *this;
		}


		// Methods

		inline Matrix3x3f& setZero()
		{
			_m00 = 0;  _m01 = 0;  _m02 = 0;
			_m10 = 0;  _m11 = 0;  _m12 = 0;
			_m20 = 0;  _m21 = 0;  _m22 = 0;
			return *this;
		}

		inline Matrix3x3f& setIdentity()
		{
			_m00 = 1.0f;  _m01 = 0;     _m02 = 0;
			_m10 = 0;     _m11 = 1.0f;  _m12 = 0;
			_m20 = 0;     _m21 = 0;     _m22 = 1.0f;
			return *this;
		}

		inline Matrix3x3f transpose() const
		{
			return Matrix3x3f(_m00, _m10, _m20,
							  _m01, _m11, _m21,
							  _m02, _m12, _m22);
		}

		inline float determinant() const
		{
			return _m00*(_m11*_m22 - _m12*_m21) - 
				   _m01*(_m01*_m22 - _m02*_m21) + 
				   _m02*(_m01*_m12 - _m02*_m11);
		}

		inline Matrix3x3f& setRow(unsigned int row, float valA, float valB, float valC)
		{
			m[row][0] = valA;
			m[row][1] = valB;
			m[row][2] = valC;
			return *this;
		}

		inline Matrix3x3f& setRow(unsigned int row, const Vector2f& vec, float val)
		{
			m[row][0] = vec.x;
			m[row][1] = vec.y;
			m[row][2] = val;
			return *this;
		}

		inline Matrix3x3f& setRow(unsigned int row, float val, const Vector2f& vec)
		{
			m[row][0] = val;
			m[row][1] = vec.x;
			m[row][2] = vec.y;
			return *this;
		}

		inline Matrix3x3f& setRow(unsigned int row, const Vector3f& vec)
		{
			m[row][0] = vec.x;
			m[row][1] = vec.y;
			m[row][2] = vec.z;
			return *this;
		}

		inline Matrix3x3f& setColumn(unsigned int column, float valA, float valB, float valC)
		{
			m[0][column] = valA;
			m[1][column] = valB;
			m[2][column] = valC;
			return *this;
		}

		inline Matrix3x3f& setColumn(unsigned int column, const Vector2f& vec, float val)
		{
			m[0][column] = vec.x;
			m[1][column] = vec.y;
			m[2][column] = val;
			return *this;
		}

		inline Matrix3x3f& setColumn(unsigned int column, float val, const Vector2f& vec)
		{
			m[0][column] = val;
			m[1][column] = vec.x;
			m[2][column] = vec.y;
			return *this;
		}

		inline Matrix3x3f& setColumn(unsigned int column, const Vector3f& vec)
		{
			m[0][column] = vec.x;
			m[1][column] = vec.y;
			m[2][column] = vec.z;
			return *this;
		}
	};
}