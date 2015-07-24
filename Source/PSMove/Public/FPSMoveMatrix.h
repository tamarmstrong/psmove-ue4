//
//  FPSMoveMatrix.h
//  A set of matrix utility methods for arbitrary sized matrices
//
//  Created by Brendan Walker on 2015-06-22.
//
#pragma once

#include <assert.h>

// FMatrixRow class needed so that we can access FMatrixMxN as a 2d array: M[i][j]  
class FMatrixRow
{
private:
	float *m_row;
	int32 m_colomn_count;

public:
	FMatrixRow(float *row, int colomn_count)
		: m_row(row), m_colomn_count(colomn_count)
	{
	}

	FORCEINLINE const float& operator[](const int colomn) const
	{
		assert(colomn > 0 && colomn < m_colomn_count);
		return m_row[colomn];
	}

	FORCEINLINE float& operator[](const int colomn)
	{
		assert(colomn > 0 && colomn < m_colomn_count);
		return m_row[colomn];
	}
};

class FMatrixMxN
{
public:
	// Default Constructor.
	// Initialize to a zero-sized matrix
	explicit FMatrixMxN();

	// Constructor. 
	// Dynamically allocates the array needed for storage.
	explicit FMatrixMxN(int32 rows, int32 colomns);

	// Constructor. 
	// assigns the C-style array used for storage.
	explicit FMatrixMxN(int32 rows, int32 colomns, float *storage);

	// If the matrix was allocated dynamically, then the destructor frees the array
	~FMatrixMxN();

	// Access the given row of the matrix
	FORCEINLINE const FMatrixRow operator[](const int row) const
	{
		assert(row > 0 && row < m_row_count);
		return FMatrixRow(&m_matrix[row*m_colomn_count], m_colomn_count);
	}

	// Access the given row of the matrix
	FORCEINLINE FMatrixRow operator[](const int row)
	{
		assert(row > 0 && row < m_row_count);
		return FMatrixRow(&m_matrix[row*m_colomn_count], m_colomn_count);
	}

	// Get the dimensions of the matrix
	FORCEINLINE int32 GetRowCount() const { return m_row_count; }
	FORCEINLINE int32 GetColomnCount() const { return m_colomn_count; }

	// Get the raw matrix array
	FORCEINLINE float *GetData() { return m_matrix; }
	FORCEINLINE const float * GetDataConst() const { return (const float *)m_matrix; }

	// Returns true of two matrices that are nearly identical
	bool IsNearlyEqual(const FMatrixMxN &matrix) const;

	// Zeros out the matrix
	void SetZero();

	// Fills the diagonals with 1s and all other elements with zero
	void SetIdentity();

	// Copies the values from another matrix. Truncates when dimensions don't match.
	void CopyFromMatrix(const FMatrixMxN &matrix);

	// Copy values from this matrix into a 4x4 unreal matrix.
	// Non-overlapping elements in the unreal matrix are left alone.
	void CopyToUnreal4x4Matrix(FMatrix &inout_matrix) const;

	// Copy values from a 4x4 unreal matrix into this one.
	// Non-overlapping elements in this matrix are left alone.
	void CopyFromUnreal4x4Matrix(const FMatrix &matrix);

	// Puts given matrix (2D array) into the Reduced Row Echelon Form.
	// Returns true if successful, False if matrix is singular.
	// Adapted from the Python code here http://elonen.iki.fi/code/misc-notes/affine-fit/
	// Which was originally written by Jarno Elonen in April 2005, released into Public Domain 
	bool ApplyGaussJordanSolver();

	// Iterative Jacobi Method for computing the Eigen values and Eigen vectors of a symmetric NxN matrix
	// - The out_eigen_values array contains the Eigen values and must be of size >= N
	// - The out_eigen_vectors Matrix contains the Eigen vectors stored in its colomns and must be of size >= NxN
	// - The out_jacobi_rotation_count returns the number of jacobi rotations performed (optional)
	bool ApplyJacobiEigenSolver(
		TArray<float> &out_eigen_values,
		FMatrixMxN &out_eigen_vectors,
		int32 *out_jacobi_rotation_count= nullptr);

private:
	// Disallow copy constructor and assignment operator
	// Not safe to do with external storage
	FMatrixMxN(const FMatrixMxN &other);
	const FMatrixMxN& operator = (const FMatrixMxN &other);

protected:
	bool m_isDynamicallyAllocated;
	float* m_matrix;
	int32 m_colomn_count;
	int32 m_row_count;
};

template <int rows, int cols>
class FStackMatrixMxN : public FMatrixMxN
{
public:
	FStackMatrixMxN() : 
		FMatrixMxN(rows, cols, (float *)m_storage)
	{ }

private:
	// Not allowed
	explicit FStackMatrixMxN(int32 rows, int32 colomns);
	explicit FStackMatrixMxN(int32 rows, int32 colomns, float *storage);

private:
	float m_storage[rows][cols];
};


// Multiplies this MxN matrix with NxO matrix resulting in an MxO matrix: C= A*B
void MatrixMultiply(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result);

// Multiplies this MxN matrix with NxO matrix accumulating to exisiting values in an MxO matrix: C+= A*B
void MatrixMultiplyAccumulate(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result);

// Adds two MxN matrices together
void MatrixAdd(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result);

// Subtracts two MxN matrices
void MatrixSubtract(
	const FMatrixMxN &a,
	const FMatrixMxN &b,
	FMatrixMxN &result);

// Copies an MxN matrix into an NxM matrix
void MatrixTranspose(
	const FMatrixMxN &matrix,
	FMatrixMxN &result);

void TestMatrixFunctions();