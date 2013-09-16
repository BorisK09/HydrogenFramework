


#pragma warning (disable:4201) // nonstandard extension used : nameless struct/union



namespace h2
{
	template <unsigned int _dimension> 
	class VectorNf
	{
	public:

		const unsigned int dimension;

		float* v;


		// Constructors

		VectorNf() : dimension(_dimension)
		{
			v = (float*)malloc(sizeof(float)*dimension);
			//v = new float[dimension];
			memset(v, 0, sizeof(float)*dimension);
		}

		VectorNf(const VectorNf &copy) : dimension(copy.dimension)
		{
			v = (float*)malloc(sizeof(float)*copy.dimension);
			//v = new float[copy.dimension];
			memset(v, 0, sizeof(float)*dimension);
			for(unsigned int i = 0; i < copy.dimension; i++)
			{
				v[i] = copy.v[i];
			}
		}


		// Destructor

		~VectorNf()
		{
			//delete[] v;
			free(v);
		}

	    
		// Copy

		VectorNf& operator = (const VectorNf &vec)
		{
			for(unsigned int i = 0; i < dimension; i++)
			{
				v[i] = vec.v[i];
			}
			return *this;
		}


		// Unary operators
	 
		VectorNf& operator + () { return *this; }

		VectorNf operator - ()
		{
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = -v[i];
			}
			return rVec;
		}


		// Binary operators

		VectorNf operator + (const VectorNf& vec) const
		{
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = v[i] + vec.v[i];
			}
			return rVec;
		}

		VectorNf operator - (const VectorNf& vec) const
		{
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = v[i] - vec.v[i];
			}
			return rVec;
		}

		VectorNf operator * (float val) const
		{
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = v[i] * val;
			}
			return rVec;
		}

		VectorNf operator * (const VectorNf& vec) const
		{
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = v[i] * vec.v[i];
			}
			return rVec;
		}

		friend VectorNf operator * (float val, const VectorNf& vec)
		{
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = val * vec.v[i];
			}
			return rVec;
		}

		VectorNf operator / (float val) const
		{
			float invVal = 1.0f/val;
			VectorNf<_dimension> rVec;
			for(unsigned int i = 0; i < dimension; i++)
			{
				rVec.v[i] = v[i] * invVal;
			}
			return rVec;
		}


		// Assigment operators

		VectorNf& operator += (const VectorNf& vec)
		{
			for(unsigned int i = 0; i < dimension; i++)
			{
				v[i] += vec.v[i];
			}
			return *this;
		}

		VectorNf& operator -= (const VectorNf& vec)
		{
			for(unsigned int i = 0; i < dimension; i++)
			{
				v[i] -= vec.v[i];
			}
			return *this;
		}

		VectorNf& operator *= (const VectorNf& vec)
		{
			for(unsigned int i = 0; i < dimension; i++)
			{
				v[i] *= vec.v[i];
			}
			return *this;
		}

		VectorNf& operator *= (float val)
		{
			for(unsigned int i = 0; i < dimension; i++)
			{
				v[i] *= val;
			}
			return *this;
		}

		VectorNf& operator /= (float val)
		{
			float invVal = 1.0f/val;
			for(unsigned int i = 0; i < dimension; i++)
			{
				v[i] *= invVal;
			}
			return *this;
		}


		// Methods

		float lenght()
		{
			float sum = 0; //v[0]^2 + v[1]^2 + ...
			for(unsigned int i = 0; i < dimension; i++)
			{
				sum += v[i]*v[i];
			}
			return sqrt(sum);
		}

		float sqlenght()
		{
			float sum = 0;
			for(unsigned int i = 0; i < dimension; i++)
			{
				sum += v[i]*v[i];
			}
			return sum;
		}

		VectorNf normalize()
		{
			float lenght = this->lenght();
			VectorNf<_dimension> rVec;
			if (lenght != 0) 
			{
				float invLenght = 1.0f/lenght;
				for(unsigned int i = 0; i < dimension; i++)
				{
					rVec.v[i] = v[i]*invLenght;
				}
				return rVec;
			} else { //return *this ?
				for(unsigned int i = 0; i < dimension; i++)
				{
					rVec.v[i] = v[i];
				}
				return rVec;
			}
		}

		float dot(const VectorNf& vec)
		{
			float sum = 0;
			for(unsigned int i = 0; i < dimension; i++)
			{
				sum += v[i] * vec.v[i]
			}
			return sum;
		}
	};
}