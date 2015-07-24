#include "PSMovePrivatePCH.h"  // Also includes psmove api lib headers.
#include "FPSMoveMatrix.h"
#include "FPSMove.h"

// -- FMatrixMxN Public Interface -- 
FMatrixMxN::FMatrixMxN()
{
	m_isDynamicallyAllocated = false;
	m_matrix = nullptr;
	m_row_count = 0;
	m_colomn_count = 0;
}

FMatrixMxN::FMatrixMxN(
	int32 rows, 
	int32 colomns, 
	float *storage)
{
	m_isDynamicallyAllocated = false;
	m_row_count = rows;
	m_colomn_count = colomns;	
	m_matrix = storage;
	memset(m_matrix, 0, sizeof(float)*rows*colomns);
}

FMatrixMxN::FMatrixMxN(
	int32 rows,
	int32 colomns)
{
	m_isDynamicallyAllocated = true;
	m_row_count = rows;
	m_colomn_count = colomns;
	m_matrix = new float[rows*colomns];
	memset(m_matrix, 0, sizeof(float)*rows*colomns);
}

FMatrixMxN::~FMatrixMxN()
{
	if (m_isDynamicallyAllocated && m_matrix != nullptr)
	{
		delete[] m_matrix;
		m_matrix = nullptr;
	}
}

// Multiplies this MxN matrix with NxO matrix resulting in an MxO matrix
void MatrixMultiply(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result)
{
	assert(&a != &result);
	assert(&b != &result);
	assert(result.GetRowCount() == a.GetRowCount());
	assert(result.GetColomnCount() == b.GetColomnCount());
	assert(a.GetColomnCount() == b.GetRowCount());

	const int32 rows = a.GetRowCount();
	const int32 colomns = b.GetColomnCount();
	const int32 shared_dimension = a.GetColomnCount();

	for (int32 row = 0; row < rows; ++row)
	{
		for (int32 col = 0; col < colomns; ++col)
		{
			result[row][col] = 0.f;

			for (int32 index = 0; index < shared_dimension; ++index)
			{
				result[row][col] += a[row][index] * b[index][col];
			}
		}
	}

}

// Multiplies this MxN matrix with NxO matrix accumulating to exisiting values in an MxO matrix
void MatrixMultiplyAccumulate(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result)
{
	assert(result.GetRowCount() == a.GetRowCount());
	assert(result.GetColomnCount() == b.GetColomnCount());
	assert(a.GetColomnCount() == b.GetRowCount());

	const int32 rows = a.GetRowCount();
	const int32 colomns = b.GetColomnCount();
	const int32 shared_dimension = a.GetColomnCount();

	for (int32 row = 0; row < rows; ++row)
	{
		for (int32 col = 0; col < colomns; ++col)
		{
			for (int32 index = 0; index < shared_dimension; ++index)
			{
				result[row][col] += a[row][index] * b[index][col];
			}
		}
	}
}

// Adds two MxN matrices together
void MatrixAdd(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result)
{
	assert(result.GetRowCount() == a.GetRowCount());
	assert(result.GetColomnCount() == a.GetColomnCount());
	assert(a.GetRowCount() == b.GetRowCount());
	assert(a.GetColomnCount() == b.GetColomnCount());

	int32 row_count = result.GetRowCount();
	int32 colomn_count = result.GetColomnCount();

	for (int32 row = 0; row < row_count; ++row)
	{
		for (int32 col = 0; col < colomn_count; ++col)
		{
			result[row][col] = a[row][col] + b[row][col];
		}
	}
}

// Subtracts two MxN matrices
void MatrixSubtract(
const FMatrixMxN &a,
const FMatrixMxN &b,
FMatrixMxN &result)
{
	assert(result.GetRowCount() == a.GetRowCount());
	assert(result.GetColomnCount() == a.GetColomnCount());
	assert(a.GetRowCount() == b.GetRowCount());
	assert(a.GetColomnCount() == b.GetColomnCount());

	int32 row_count = result.GetRowCount();
	int32 colomn_count = result.GetColomnCount();

	for (int32 row = 0; row < row_count; ++row)
	{
		for (int32 col = 0; col < colomn_count; ++col)
		{
			result[row][col] = a[row][col] - b[row][col];
		}
	}
}

void MatrixTranspose(
	const FMatrixMxN &matrix,
	FMatrixMxN &result)
{
	assert(result.GetRowCount() == matrix.GetColomnCount());
	assert(result.GetColomnCount() == matrix.GetRowCount());

	int32 row_count = result.GetRowCount();
	int32 colomn_count = result.GetColomnCount();

	for (int32 row = 0; row < row_count; ++row)
	{
		for (int32 col = 0; col < colomn_count; ++col)
		{
			result[row][col] = matrix[col][row];
		}
	}
}

// Returns true of two matrices that are nearly identical
bool FMatrixMxN::IsNearlyEqual(
	const FMatrixMxN &other) const
{
	bool isNearlyEqual = false;

	if (this->m_row_count == other.m_row_count && 
		this->m_colomn_count == other.m_colomn_count)
	{
		const FMatrixMxN& m = *this;

		for (int32 row = 0; row < this->m_row_count && isNearlyEqual; ++row)
		{
			for (int32 col = 0; col < this->m_colomn_count && isNearlyEqual; ++col)
			{
				isNearlyEqual = FMath::IsNearlyEqual(m[row][col], other[row][col], FLOAT_NORMAL_THRESH);
			}
		}
	}

	return isNearlyEqual;
}

void FMatrixMxN::SetZero()
{
	if (m_matrix != nullptr)
	{
		memset(m_matrix, 0, sizeof(float)*m_colomn_count*m_row_count);
	}
}

void FMatrixMxN::SetIdentity()
{
	assert(m_row_count == m_colomn_count);
	FMatrixMxN& m = *this;

	for (int32 row = 0; row < m_row_count; ++row)
	{
		FMatrixRow thisRow = m[row];

		for (int32 col = 0; col < m_colomn_count; ++col)
		{
			thisRow[col] = (row == col) ? 1.f : 0.f;
		}
	}
}

// Copies the values from a matrix of greater or equal size
void FMatrixMxN::CopyFromMatrix(const FMatrixMxN &matrix)
{
	int32 row_count = FMath::Min(m_row_count, matrix.GetRowCount());
	int32 colomn_count = FMath::Min(m_colomn_count, matrix.GetColomnCount());
	FMatrixMxN& m = *this;

	for (int32 row = 0; row < row_count; ++row)
	{
		FMatrixRow thisRow = m[row];
		FMatrixRow otherRow = matrix[row];

		for (int32 col = 0; col < colomn_count; ++col)
		{
			thisRow[col] = otherRow[col];
		}
	}
}

// Copy values from this matrix into a 4x4 unreal matrix.
// Non-overlapping elements in the unreal matrix are left alone.
void FMatrixMxN::CopyToUnreal4x4Matrix(
	FMatrix &inout_matrix) const
{
	int32 row_count = FMath::Min(m_row_count, 4);
	int32 colomn_count = FMath::Min(m_colomn_count, 4);
	const FMatrixMxN& m = *this;

	for (int32 row = 0; row < row_count; ++row)
	{
		const FMatrixRow thisRow = m[row];

		for (int32 col = 0; col < colomn_count; ++col)
		{
			inout_matrix.M[row][col] = thisRow[col];
		}
	}
}

// Copy values from a 4x4 unreal matrix into this one.
// Non-overlapping elements in this matrix are left alone.
void FMatrixMxN::CopyFromUnreal4x4Matrix(
	const FMatrix &matrix)
{
	int32 row_count = FMath::Min(m_row_count, 4);
	int32 colomn_count = FMath::Min(m_colomn_count, 4);
	FMatrixMxN& m = *this;

	for (int32 row = 0; row < row_count; ++row)
	{
		FMatrixRow thisRow= m[row];

		for (int32 col = 0; col < colomn_count; ++col)
		{
			thisRow[col] = matrix.M[row][col];
		}
	}
}

// Ultra simple linear system solver. Replace this if you need speed.
// Puts given matrix (2D array) into the Reduced Row Echelon Form.
// Returns true if successful, False if matrix is singular.
// Adapted from the Python code here http://elonen.iki.fi/code/misc-notes/affine-fit/
// Which was originally written by Jarno Elonen in April 2005, released into Public Domain 
bool FMatrixMxN::ApplyGaussJordanSolver()
{
	const int rows = this->GetRowCount();
	const int colomns = this->GetColomnCount();
	const float eps = FLT_EPSILON;
	FMatrixMxN& m = *this;

	for (int32 row = 0; row < rows; row++)
	{
		int max_row = row;

		for (int32 other_row = row + 1; other_row < rows; other_row++) // Find max pivot
		{
			if (FMath::Abs(m[other_row][row]) > FMath::Abs(m[max_row][row]))
			{
				max_row = other_row;
			}
		}

		// Swap row with max_row
		{
			FMatrixRow m_row = m[row];
			FMatrixRow m_max_row = m[max_row];

			for (int32 col = 0; col < colomns; ++col)
			{
				float temp = m_row[col];

				m_row[col] = m_max_row[col];
				m_max_row[col] = temp;
			}
		}

		if (FMath::Abs(m[row][row]) <= eps) // Singular?
		{
			return false;
		}

		for (int32 other_row = row + 1; other_row < rows; other_row++) // Eliminate column y
		{
			float c = m[other_row][row] / m[row][row]; // Singularity check above verifies this is safe

			for (int colomn = row; colomn < colomns; colomn++)
			{
				m[other_row][colomn] -= m[row][colomn] * c;
			}
		}
	}

	for (int32 row = rows - 1; row > -1; row--) // Backsubstitute
	{
		float c = m[row][row];

		for (int32 other_row = 0; other_row < row; other_row++)
		{
			for (int32 colomn = colomns - 1; colomn > row - 1; colomn--)
			{
				m[other_row][colomn] -= m[row][colomn] * m[other_row][row] / c;
			}
		}

		m[row][row] /= c;

		for (int colomn = rows; colomn < colomns; colomn++) // Normalize row
		{
			m[row][colomn] /= c;
		}
	}

	return true;
}

/**
* Adapted from "Numeric Recipes in C", pg 467 (Jacobi Transformations of a Symmetric Matrix)
* Computes all Eigen-values and Eigen-vectors of a real symmetric matrix a[n][n].
* [Outputs]
*  - Elements of this matrix above the diagonal are destroyed.
*  - Vector 'out_eigen_vectors' returns the eigenvalues.
*  - The colomns matrix out_eigen_vectors[n][n] contain the normalized Eigen-vectors.
*  - out_jacobi_rotation_count returns the number of Jacobi rotations performed
*/

inline void jacobi_rotate(
	const int32 row_i,
	const int32 col_j,
	const int32 row_k,
	const int32 col_l,
	const float tau, // See Eqn 11.1.18
	const float s, // See Eqn 11.1.11 and 11.1.12
	FMatrixMxN &symmetric_matrix, // inout
	float &g, // inout
	float &h) // inout
{
	g = symmetric_matrix[row_i][col_j];
	h = symmetric_matrix[row_k][col_l];
	symmetric_matrix[row_i][col_j] = g - s*(h + g*tau);
	symmetric_matrix[row_k][col_l] = h + s*(g - h*tau);
}

bool FMatrixMxN::ApplyJacobiEigenSolver(
	TArray<float> &out_eigen_values,
	FMatrixMxN &out_eigen_vectors,
	int32 *out_jacobi_rotation_count)
{
	assert(m_colomn_count == m_row_count);
	assert(out_eigen_vectors.GetRowCount() >= this->GetRowCount());
	assert(out_eigen_vectors.GetColomnCount() >= this->GetColomnCount());

	static const int32 MAX_ITERATION_COUNT = 50;
	const int32 matrix_size = this->GetColomnCount();

	FMatrixMxN &symmetric_matrix = *this;
	int32 jacobi_rotation_count = 0;
	bool success = false;

	TArray<float> b;
	TArray<float> z;

	b.SetNumZeroed(matrix_size);
	z.SetNumZeroed(matrix_size);
	out_eigen_values.SetNumZeroed(m_colomn_count);

	// Initialize the Eigen-vector matrix to the identity matrix
	for (int32 ip = 0; ip < matrix_size; ip++)
	{
		for (int32 iq = 0; iq < matrix_size; iq++)
		{
			out_eigen_vectors[ip][iq] = 0.0;
		}

		out_eigen_vectors[ip][ip] = 1.0;
	}

	for (int32 ip = 0; ip < matrix_size; ip++)
	{
		// Initialize vector 'b' and the eigen-values vector to the diagonals of the symmetric matrix 
		b[ip] = out_eigen_values[ip] = symmetric_matrix[ip][ip];

		// The 'z' vector will accumulate terms of the form: t*symmetric_matrix[p][q]
		// where:
		//  t = sgn(theta) / (abs(theta) + sqrt(theta^2 + 1))
		//  theta = (symmetric_matrix[q][q] - symmetric_matrix[p][p]) / (2*symmetric_matrix[p][q])
		// (See Eqn 11.1.14, 11.1.10, and 11.1.8 in "Numeric Recipes in C", pg. 467-468)
		z[ip] = 0.0;
	}

	for (int32 i = 0; i < MAX_ITERATION_COUNT; i++)
	{
		// Compute the sum of the off diagonal elements
		float off_diagonal_sum = 0.f;

		for (int32 ip = 0; ip < matrix_size - 1; ip++)
		{
			for (int32 iq = ip + 1; iq < matrix_size; iq++)
			{
				off_diagonal_sum += FMath::Abs(symmetric_matrix[ip][iq]);
			}
		}

		if (off_diagonal_sum == 0.0)
		{
			// The normal termination condition.
			// This relies on quadratic convergence to machine underflow.
			success = true;
			break;
		}

		// Threshold non-zero for first three sweeps only
		const float threshold = (i < 4) ? (0.2*off_diagonal_sum / (matrix_size*matrix_size)) : 0.f;

		for (int32 ip = 0; ip < matrix_size - 1; ip++)
		{
			for (int32 iq = ip + 1; iq < matrix_size; iq++)
			{
				float g = 100.0*FMath::Abs(symmetric_matrix[ip][iq]);

				if (i > 4 &&
					(FMath::Abs(out_eigen_values[ip]) + g) == FMath::Abs(out_eigen_values[ip]) &&
					(FMath::Abs(out_eigen_values[iq]) + g) == FMath::Abs(out_eigen_values[iq]))
				{
					symmetric_matrix[ip][iq] = 0.0f;
				}
				else if (FMath::Abs(symmetric_matrix[ip][iq]) > threshold)
				{
					float t;

					{
						float h = out_eigen_values[iq] - out_eigen_values[ip];

						if ((FMath::Abs(h) + g) == FMath::Abs(h))
						{
							t = (symmetric_matrix[ip][iq]) / h; // t = 1/(2*theta)
						}
						else
						{
							// See Eqn 11.1.10 in "Numeric Recipes in C", pg. 467-468)
							const float theta = 0.5f*h / (symmetric_matrix[ip][iq]);

							t = 1.f / (FMath::Abs(theta) + FMath::Sqrt(1.0f + theta*theta));

							if (theta < 0.f)
							{
								t = -t;
							}
						}
					}

					{
						const float c = 1.f / FMath::Sqrt(1.f + t*t);
						const float s = t*c;
						const float tau = s / (1.f + c);
						float h = t*symmetric_matrix[ip][iq];

						z[ip] -= h;
						z[iq] += h;
						out_eigen_values[ip] -= h;
						out_eigen_values[iq] += h;
						symmetric_matrix[ip][iq] = 0.f;

						// Case of rotations i <= j < p
						for (int32 j = 0; j <= ip - 1; j++)
						{
							jacobi_rotate(
								j, ip, j, iq, // row, col, row, col
								tau, s, // rotation constants
								symmetric_matrix, g, h); // inout parameters
						}

						// Case of rotations p <= j < q
						for (int32 j = ip + 1; j <= iq - 1; j++)
						{
							jacobi_rotate(
								ip, j, j, iq, // row, col, row, col
								tau, s, // rotation constants
								symmetric_matrix, g, h); // inout parameters
						}

						// Case of rotations q <= j < n
						for (int32 j = iq + 1; j < matrix_size; j++)
						{
							jacobi_rotate(
								ip, j, iq, j, // row, col, row, col
								tau, s, // rotation constants
								symmetric_matrix, g, h); // inout parameters
						}

						for (int32 j = 0; j < matrix_size; j++)
						{
							jacobi_rotate(
								j, ip, j, iq, // row, col, row, col
								tau, s, // rotation constants
								out_eigen_vectors, g, h); // inout parameters
						}
					}

					++jacobi_rotation_count;
				}
			}
		}

		for (int32 ip = 0; ip < matrix_size; ip++)
		{
			b[ip] += z[ip];

			// Update Eigen-values with the sum of t*symmetric_matrix[p][q]
			out_eigen_values[ip] = b[ip];

			// Reset 'z' vector
			z[ip] = 0.0;
		}
	}

	if (!success)
	{
		UE_LOG(LogPSMove, Error, TEXT("Too many iterations in routine jacobi"));
	}

	if (out_jacobi_rotation_count != nullptr)
	{
		*out_jacobi_rotation_count = jacobi_rotation_count;
	}

	return success;
}

// -- Unit Tests ------
static void TestJacobiEigenSolver()
{
	float _symmetric_matrix[4][4] = {
		{ 2, -1, 0, 3 },
		{ -1, 1, 4, 1 },
		{ 0, 4, 1, 2 },
		{ 3, 1, 2, 1 }
	};
	FMatrixMxN symmetric_matrix(4, 4, (float *)_symmetric_matrix);

	float _actual_eigen_vectors[4][4];
	FMatrixMxN actual_eigen_vectors(4, 4, (float *)_actual_eigen_vectors);

	TArray<float> actual_eigen_values;

	float _desired_eigen_vectors[4][4] = {
		{ 0.763540f, 0.229646f, -0.034860f, -0.602539f },
		{ -0.414868f, 0.551394f, -0.668720f, -0.276880f },
		{ -0.249993f, 0.631331f, 0.724554f, -0.118093f },
		{ 0.427076f, 0.494620f, -0.163155f, 0.739147f }
	};
	FMatrixMxN desired_eigen_vectors(4, 4, (float *)_desired_eigen_vectors);

	bool success= symmetric_matrix.ApplyJacobiEigenSolver(actual_eigen_values, actual_eigen_vectors);
	assert(success);

	// Make sure we got the eigen values and vectors we expected
	success = FMath::IsNearlyEqual(actual_eigen_values[0], 4.2213586f, FLOAT_NORMAL_THRESH);
	assert(success);

	success = FMath::IsNearlyEqual(actual_eigen_values[1], 6.06044327f, FLOAT_NORMAL_THRESH);
	assert(success);

	success = FMath::IsNearlyEqual(actual_eigen_values[2], -3.142122f, FLOAT_NORMAL_THRESH);
	assert(success);

	success = FMath::IsNearlyEqual(actual_eigen_values[3], -2.139680f, FLOAT_NORMAL_THRESH);
	assert(success);

	success &= desired_eigen_vectors.IsNearlyEqual(actual_eigen_vectors);
	assert(success);
}

void TestMatrixFunctions()
{
	TestJacobiEigenSolver();
}