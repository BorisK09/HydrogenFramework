


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	class Matrix4x4f
	{
	public:

		static const unsigned int nRows = 4, nColumns = 4;

		union
		{
			struct
			{
				float _m00, _m01, _m02, _m03,
					  _m10, _m11, _m12, _m13,
					  _m20, _m21, _m22, _m23,
					  _m30, _m31, _m32, _m33;
			};

			float m[4][4];

			float mv[16];
		};


		// Constructors

		Matrix4x4f() : _m00(0), _m01(0), _m02(0), _m03(0),
					   _m10(0), _m11(0), _m12(0), _m13(0),
					   _m20(0), _m21(0), _m22(0), _m23(0),
					   _m30(0), _m31(0), _m32(0), _m33(0) {}

		Matrix4x4f(float in_m00, float in_m01, float in_m02, float in_m03,
				   float in_m10, float in_m11, float in_m12, float in_m13,
				   float in_m20, float in_m21, float in_m22, float in_m23,
				   float in_m30, float in_m31, float in_m32, float in_m33) : 
									_m00(in_m00), _m01(in_m01), _m02(in_m02), _m03(in_m03),
									_m10(in_m10), _m11(in_m11), _m12(in_m12), _m13(in_m13),
									_m20(in_m20), _m21(in_m21), _m22(in_m22), _m23(in_m23),
									_m30(in_m30), _m31(in_m31), _m32(in_m32), _m33(in_m33) {}

		template <class MatType>
		Matrix4x4f(MatType const &mat) : _m00(mat.m[0][0]), _m01(mat.m[0][1]), _m02(mat.m[0][2]), _m03(mat.m[0][3]),
										 _m10(mat.m[1][0]), _m11(mat.m[1][1]), _m12(mat.m[1][2]), _m13(mat.m[1][3]),
										 _m20(mat.m[2][0]), _m21(mat.m[2][1]), _m22(mat.m[2][2]), _m23(mat.m[2][3]),
										 _m30(mat.m[3][0]), _m31(mat.m[3][1]), _m22(mat.m[3][2]), _m23(mat.m[3][3]) {}
	 

		// Copy

		inline Matrix4x4f& operator = (const Matrix4x4f &mat)
		{
			_m00 = mat._m00;  _m01 = mat._m01;  _m02 = mat._m02; _m03 = mat._m03;
			_m10 = mat._m10;  _m11 = mat._m11;  _m12 = mat._m12; _m13 = mat._m13;
			_m20 = mat._m20;  _m21 = mat._m21;  _m22 = mat._m22; _m22 = mat._m22;
			_m30 = mat._m30;  _m31 = mat._m31;  _m32 = mat._m32; _m33 = mat._m33;

			return *this;
		}

		inline Matrix4x4f& operator = (const Matrix3x3f &mat)
		{
			_m00 = mat._m00;  _m01 = mat._m01;  _m02 = mat._m02;
			_m10 = mat._m10;  _m11 = mat._m11;  _m12 = mat._m12;
			_m20 = mat._m20;  _m21 = mat._m21;  _m22 = mat._m22;

			return *this;
		}

		inline Matrix4x4f& operator = (const Matrix2x2f &mat)
		{
			_m00 = mat._m00;  _m01 = mat._m01;
			_m10 = mat._m10;  _m11 = mat._m11;

			return *this;
		}


		// Unary operators

		inline Matrix4x4f operator + () const { return *this; }

		inline Matrix4x4f operator - () const
		{
			return Matrix4x4f(-_m00, -_m01, -_m02, -_m03,
							  -_m10, -_m11, -_m12, -_m13,
							  -_m20, -_m21, -_m22, -_m23,
							  -_m30, -_m31, -_m32, -_m33);
		}


		// Binary operators

		inline Matrix4x4f operator + (const Matrix4x4f& mat) const
		{
			return Matrix4x4f(_m00 + mat._m00,  _m01 + mat._m01,  _m02 + mat._m02, _m03 + mat._m03,
							  _m10 + mat._m10,  _m11 + mat._m11,  _m12 + mat._m12, _m13 + mat._m13,
							  _m20 + mat._m20,  _m21 + mat._m21,  _m22 + mat._m22, _m23 + mat._m23,
							  _m30 + mat._m30,  _m31 + mat._m31,  _m32 + mat._m32, _m33 + mat._m33);
		}

		inline Matrix4x4f operator - (const Matrix4x4f& mat) const
		{
			return Matrix4x4f(_m00 - mat._m00,  _m01 - mat._m01,  _m02 - mat._m02, _m03 - mat._m03,
							  _m10 - mat._m10,  _m11 - mat._m11,  _m12 - mat._m12, _m13 - mat._m13,
							  _m20 - mat._m20,  _m21 - mat._m21,  _m22 - mat._m22, _m23 - mat._m23,
							  _m30 - mat._m30,  _m31 - mat._m31,  _m32 - mat._m32, _m33 - mat._m33);
		}

		inline Matrix4x4f operator * (float val) const
		{
			return Matrix4x4f(_m00*val, _m01*val, _m02*val, _m03*val,
							  _m10*val, _m11*val, _m12*val, _m13*val,
							  _m20*val, _m21*val, _m22*val, _m23*val,
							  _m30*val, _m31*val, _m32*val, _m33*val);
		}

		friend Matrix4x4f operator * (float val, const Matrix4x4f& mat)
		{
			return Matrix4x4f(mat._m00*val, mat._m01*val, mat._m02*val, mat._m03*val,
							  mat._m10*val, mat._m11*val, mat._m12*val, mat._m13*val,
							  mat._m20*val, mat._m21*val, mat._m22*val, mat._m23*val,
							  mat._m30*val, mat._m31*val, mat._m32*val, mat._m33*val);
		}

		inline Vector4f operator * (const Vector4f& vec) const
		{
			return Vector4f(_m00*vec.x + _m01*vec.y + _m02*vec.z + _m03*vec.w,
							_m10*vec.x + _m11*vec.y + _m12*vec.z + _m13*vec.w,
							_m20*vec.x + _m21*vec.y + _m22*vec.z + _m23*vec.w,
							_m30*vec.x + _m31*vec.y + _m32*vec.z + _m33*vec.w);
		}

		friend Vector4f operator * (const Vector4f& vec, const Matrix4x4f& mat)
		{
			return Vector4f(vec.x*mat._m00 + vec.y*mat._m10 + vec.z*mat._m20 + vec.w*mat._m30,
							vec.x*mat._m01 + vec.y*mat._m11 + vec.z*mat._m21 + vec.w*mat._m31,
							vec.x*mat._m02 + vec.y*mat._m12 + vec.z*mat._m22 + vec.w*mat._m32,
							vec.x*mat._m03 + vec.y*mat._m13 + vec.z*mat._m23 + vec.w*mat._m33);
		}

		inline Matrix4x4f operator * (const Matrix4x4f& mat) const
		{
			Matrix4x4f rMat;

			rMat._m00 = _m00*mat._m00 + _m01*mat._m10 + _m02*mat._m20 + _m03*mat._m30;
			rMat._m01 = _m00*mat._m01 + _m01*mat._m11 + _m02*mat._m21 + _m03*mat._m31;
			rMat._m02 = _m00*mat._m02 + _m01*mat._m12 + _m02*mat._m22 + _m03*mat._m32;
			rMat._m03 = _m00*mat._m03 + _m01*mat._m13 + _m02*mat._m23 + _m03*mat._m33;
			rMat._m10 = _m10*mat._m00 + _m11*mat._m10 + _m12*mat._m20 + _m13*mat._m30;
			rMat._m11 = _m10*mat._m01 + _m11*mat._m11 + _m12*mat._m23 + _m13*mat._m31;
			rMat._m12 = _m10*mat._m02 + _m11*mat._m12 + _m12*mat._m22 + _m13*mat._m32;
			rMat._m13 = _m10*mat._m03 + _m11*mat._m13 + _m12*mat._m23 + _m13*mat._m33;
			rMat._m20 = _m20*mat._m00 + _m21*mat._m10 + _m22*mat._m20 + _m23*mat._m30;
			rMat._m21 = _m20*mat._m01 + _m21*mat._m11 + _m22*mat._m21 + _m23*mat._m31;
			rMat._m22 = _m20*mat._m02 + _m21*mat._m12 + _m22*mat._m22 + _m23*mat._m32;
			rMat._m23 = _m20*mat._m03 + _m21*mat._m13 + _m22*mat._m23 + _m23*mat._m33;
			rMat._m30 = _m30*mat._m00 + _m31*mat._m10 + _m32*mat._m20 + _m33*mat._m30;
			rMat._m31 = _m30*mat._m01 + _m31*mat._m11 + _m32*mat._m21 + _m33*mat._m31;
			rMat._m32 = _m30*mat._m02 + _m31*mat._m12 + _m32*mat._m22 + _m33*mat._m32;
			rMat._m33 = _m30*mat._m03 + _m31*mat._m13 + _m32*mat._m22 + _m33*mat._m33;

			return rMat;
		}

		inline Matrix4x4f operator / (float val) const
		{
			float invVal = 1.0f/val;
			return Matrix4x4f(_m00*invVal, _m01*invVal, _m02*invVal, _m03*invVal,
							  _m10*invVal, _m11*invVal, _m12*invVal, _m13*invVal,
							  _m20*invVal, _m21*invVal, _m22*invVal, _m23*invVal,
							  _m30*invVal, _m31*invVal, _m32*invVal, _m33*invVal);
		}


		// Assigment operators

		inline Matrix4x4f& operator += (const Matrix4x4f& mat)
		{
			_m00 += mat._m00;  _m01 += mat._m01;  _m02 += mat._m02;  _m03 += mat._m03;
			_m10 += mat._m10;  _m11 += mat._m11;  _m12 += mat._m12;  _m13 += mat._m13;
			_m20 += mat._m20;  _m21 += mat._m21;  _m22 += mat._m22;  _m23 += mat._m23;
			_m30 += mat._m30;  _m31 += mat._m31;  _m32 += mat._m32;  _m33 += mat._m33;

			return *this;
		}

		inline Matrix4x4f& operator -= (const Matrix4x4f& mat)
		{
			_m00 -= mat._m00;  _m01 -= mat._m01;  _m02 -= mat._m02;  _m03 -= mat._m03;
			_m10 -= mat._m10;  _m11 -= mat._m11;  _m12 -= mat._m12;  _m13 -= mat._m13;
			_m20 -= mat._m20;  _m21 -= mat._m21;  _m22 -= mat._m22;  _m23 -= mat._m23;
			_m30 -= mat._m30;  _m31 -= mat._m31;  _m32 -= mat._m32;  _m33 -= mat._m33;

			return *this;
		}

		inline Matrix4x4f& operator *= (float val)
		{
			_m00 *= val;  _m01 *= val;  _m02 *= val;  _m03 *= val;
			_m10 *= val;  _m11 *= val;  _m12 *= val;  _m13 *= val;
			_m20 *= val;  _m21 *= val;  _m22 *= val;  _m23 *= val;
			_m30 *= val;  _m31 *= val;  _m32 *= val;  _m33 *= val;

			return *this;
		}

		inline Matrix4x4f& operator *= (const Matrix4x4f& mat)
		{
			_m00 = _m00*mat._m00 + _m01*mat._m10 + _m02*mat._m20 + _m03*mat._m30;
			_m01 = _m00*mat._m01 + _m01*mat._m11 + _m02*mat._m21 + _m03*mat._m31;
			_m02 = _m00*mat._m02 + _m01*mat._m12 + _m02*mat._m22 + _m03*mat._m32;
			_m03 = _m00*mat._m03 + _m01*mat._m13 + _m02*mat._m23 + _m03*mat._m33;
			_m10 = _m10*mat._m00 + _m11*mat._m10 + _m12*mat._m20 + _m13*mat._m30;
			_m11 = _m10*mat._m01 + _m11*mat._m11 + _m12*mat._m23 + _m13*mat._m31;
			_m12 = _m10*mat._m02 + _m11*mat._m12 + _m12*mat._m22 + _m13*mat._m32;
			_m13 = _m10*mat._m03 + _m11*mat._m13 + _m12*mat._m23 + _m13*mat._m33;
			_m20 = _m20*mat._m00 + _m21*mat._m10 + _m22*mat._m20 + _m23*mat._m30;
			_m21 = _m20*mat._m01 + _m21*mat._m11 + _m22*mat._m21 + _m23*mat._m31;
			_m22 = _m20*mat._m02 + _m21*mat._m12 + _m22*mat._m22 + _m23*mat._m32;
			_m23 = _m20*mat._m03 + _m21*mat._m13 + _m22*mat._m23 + _m23*mat._m33;
			_m30 = _m30*mat._m00 + _m31*mat._m10 + _m32*mat._m20 + _m33*mat._m30;
			_m31 = _m30*mat._m01 + _m31*mat._m11 + _m32*mat._m21 + _m33*mat._m31;
			_m32 = _m30*mat._m02 + _m31*mat._m12 + _m32*mat._m22 + _m33*mat._m32;
			_m33 = _m30*mat._m03 + _m31*mat._m13 + _m32*mat._m22 + _m33*mat._m33;

			return *this;
		}

		inline Matrix4x4f& operator /= (float val)
		{
			float invVal = 1.0f/val;

			_m00 *= invVal;  _m01 *= invVal;  _m02 *= invVal;  _m03 *= invVal;
			_m10 *= invVal;  _m11 *= invVal;  _m12 *= invVal;  _m13 *= invVal;
			_m20 *= invVal;  _m21 *= invVal;  _m22 *= invVal;  _m23 *= invVal;
			_m30 *= invVal;  _m31 *= invVal;  _m32 *= invVal;  _m33 *= invVal;

			return *this;
		}


		// Methods

		inline Matrix4x4f& setZero()
		{
			_m00 = 0;  _m01 = 0;  _m02 = 0;  _m03 = 0;
			_m10 = 0;  _m11 = 0;  _m12 = 0;  _m13 = 0;
			_m20 = 0;  _m21 = 0;  _m22 = 0;  _m23 = 0;
			_m30 = 0;  _m31 = 0;  _m32 = 0;  _m33 = 0;

			return *this;
		}

		inline Matrix4x4f& setIdentity()
		{
			_m00 = 1.0f;  _m01 = 0;     _m02 = 0;     _m03 = 0;
			_m10 = 0;     _m11 = 1.0f;  _m12 = 0;     _m13 = 0;
			_m20 = 0;     _m21 = 0;     _m22 = 1.0f;  _m23 = 0;
			_m30 = 0;     _m31 = 0;     _m32 = 0;     _m33 = 1.0f;

			return *this;
		}

		inline Matrix4x4f transpose() const
		{
			return Matrix4x4f(_m00, _m10, _m20, _m30,
							  _m01, _m11, _m21, _m31,
							  _m02, _m12, _m22, _m32,
							  _m03, _m13, _m23, _m33);
		}

		inline float determinant() const
		{
			return _m00*(Matrix3x3f(_m11, _m21, _m31,
									_m12, _m22, _m32,
									_m13, _m23, _m33).determinant()) - 

				   _m01*(Matrix3x3f(_m01, _m21, _m31,
									_m02, _m22, _m32,
									_m03, _m23, _m33).determinant()) + 

				   _m02*(Matrix3x3f(_m01, _m11, _m31,
									_m02, _m12, _m32,
									_m03, _m13, _m33).determinant()) -

				   _m03*(Matrix3x3f(_m01, _m11, _m21,
									_m02, _m12, _m22,
									_m03, _m13, _m23).determinant());
		}

		inline Matrix4x4f& setRow(unsigned int row, float valA, float valB, float valC, float valD)
		{
			m[row][0] = valA;
			m[row][1] = valB;
			m[row][2] = valC;
			m[row][3] = valD;

			return *this;
		}

		inline Matrix4x4f& setRow(unsigned int row, const Vector2f& vec, float valA, float valB)
		{
			m[row][0] = vec.x;
			m[row][1] = vec.y;
			m[row][2] = valA;
			m[row][3] = valB;

			return *this;
		}

		inline Matrix4x4f& setRow(unsigned int row, float valA, const Vector2f& vec, float valB)
		{
			m[row][0] = valA;
			m[row][1] = vec.x;
			m[row][2] = vec.y;
			m[row][3] = valB;

			return *this;
		}

		inline Matrix4x4f& setRow(unsigned int row, float valA, float valB, const Vector2f& vec)
		{
			m[row][0] = valA;
			m[row][1] = valB;
			m[row][2] = vec.x;
			m[row][3] = vec.y;

			return *this;
		}

		inline Matrix4x4f& setRow(unsigned int row, const Vector3f& vec, float val)
		{
			m[row][0] = vec.x;
			m[row][1] = vec.y;
			m[row][2] = vec.z;
			m[row][3] = val;

			return *this;
		}

		inline Matrix4x4f& setRow(unsigned int row, float val, const Vector3f& vec)
		{
			m[row][0] = val;
			m[row][1] = vec.x;
			m[row][2] = vec.y;
			m[row][3] = vec.z;

			return *this;
		}

		inline Matrix4x4f& setRow(unsigned int row, const Vector4f& vec)
		{
			m[row][0] = vec.x;
			m[row][1] = vec.y;
			m[row][2] = vec.z;
			m[row][3] = vec.w;

			return *this;
		}

	///////////////

		inline Matrix4x4f& setColumn(unsigned int column, float valA, float valB, float valC, float valD)
		{
			m[0][column] = valA;
			m[1][column] = valB;
			m[2][column] = valC;
			m[3][column] = valD;

			return *this;
		}

		inline Matrix4x4f& setColumn(unsigned int column, const Vector2f& vec, float valA, float valB)
		{
			m[0][column] = vec.x;
			m[1][column] = vec.y;
			m[2][column] = valA;
			m[3][column] = valB;

			return *this;
		}

		inline Matrix4x4f& setColumn(unsigned int column, float valA, const Vector2f& vec, float valB)
		{
			m[0][column] = valA;
			m[1][column] = vec.x;
			m[2][column] = vec.y;
			m[3][column] = valB;

			return *this;
		}

		inline Matrix4x4f& setColumn(unsigned int column, float valA, float valB, const Vector2f& vec)
		{
			m[0][column] = valA;
			m[1][column] = valB;
			m[2][column] = vec.x;
			m[3][column] = vec.y;

			return *this;
		}

		inline Matrix4x4f& setColumn(unsigned int column, const Vector3f& vec, float val)
		{
			m[0][column] = vec.x;
			m[1][column] = vec.y;
			m[2][column] = vec.z;
			m[3][column] = val;

			return *this;
		}

		inline Matrix4x4f& setColumn(unsigned int column, float val, const Vector3f& vec)
		{
			m[0][column] = val;
			m[1][column] = vec.x;
			m[2][column] = vec.y;
			m[3][column] = vec.z;

			return *this;
		}

		inline Matrix4x4f& setColumn(unsigned int column, const Vector4f& vec)
		{
			m[0][column] = vec.x;
			m[1][column] = vec.y;
			m[2][column] = vec.z;
			m[3][column] = vec.w;

			return *this;
		}
	};
}