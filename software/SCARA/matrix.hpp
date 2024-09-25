#ifndef MATRIX_HPP
#define MATRIX_HPP

template <typename T>
class Matrix {
private:
    T* data;           // 1D array to store matrix elements
    size_t rows;       // Number of rows
    size_t cols;       // Number of columns

public:
    // Constructor
    Matrix(size_t rows, size_t cols) : rows(rows), cols(cols) {
        data = new T[rows * cols](); // Initialize with zeros
    }

    // Copy constructor
    Matrix(const Matrix& other) : rows(other.rows), cols(other.cols) {
        data = new T[rows * cols];
        for (size_t i = 0; i < rows * cols; ++i) {
            data[i] = other.data[i];
        }
    }

    // Move constructor
    Matrix(Matrix&& other) : data(other.data), rows(other.rows), cols(other.cols) {
        other.data = nullptr; // Invalidate the source
    }

    // Destructor
    ~Matrix() {
        delete[] data;
    }

    // Operator () to access elements
    T& operator()(size_t row, size_t col) {
        return data[row * cols + col];
    }

    const T& operator()(size_t row, size_t col) const {
        return data[row * cols + col];
    }

    // Transpose the matrix
    Matrix<T> transpose() const {
        Matrix<T> result(cols, rows);
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < cols; ++c) {
                result(c, r) = (*this)(r, c);
            }
        }
        return result;
    }

    // Multiply two matrices
    Matrix<T> multiply(const Matrix<T>& other) const {
        if (cols != other.rows) {
            // Handle multiplication error
            return Matrix<T>(0, 0); // Return an empty matrix
        }
        Matrix<T> result(rows, other.cols);
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < other.cols; ++c) {
                result(r, c) = 0;
                for (size_t k = 0; k < cols; ++k) {
                    result(r, c) += (*this)(r, k) * other(k, c);
                }
            }
        }
        return result;
    }

    // Multiply the matrix by a vector
    // Suppose the size of the vector matches the number of columns of the matrix
    Vector<T> multiply(const Vector<T> vector) const {
        Vector<T> result(rows); // Resulting vector (column matrix)
        for (size_t r = 0; r < rows; ++r) {
            result[r] = 0;
            for (size_t c = 0; c < cols; ++c) {
                result[r] += (*this)(r, c) * vector[c];
            }
        }
        return result;
    }
    
    // Get a submatrix by excluding the specified row and column
    Matrix<T> getSubMatrix(size_t excludeRow, size_t excludeCol) const {
        Matrix<T> subMatrix(rows - 1, cols - 1);
        size_t subRow = 0, subCol = 0;
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < cols; ++c) {
                if (r != excludeRow && c != excludeCol) {
                    subMatrix(subRow, subCol++) = (*this)(r, c);
                }
            }
            if (r != excludeRow) {
                subRow++;
                subCol = 0;
            }
        }
        return subMatrix;
    }
    
    // Compute the determinant of the matrix
    T determinant() const {
        if (rows != cols) {
            return T(0); // Return 0 for non-square
        }

        // Base cases for 1x1 and 2x2 matrices
        if (rows == 1) {
            return (*this)(0, 0);
        } else if (rows == 2) {
            return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
        }

        // Recursive case for larger matrices
        T det = 0;
        for (size_t c = 0; c < cols; ++c) {
            det += (*this)(0, c) * ((c % 2 == 0) ? 1 : -1) * getSubMatrix(0, c).determinant();
        }
        return det;
    }
    
    // Compute the inverse of the matrix
    Matrix<T> inverse() const {

        if (rows != cols) {
            return Matrix<T>(0, 0); // Return an empty matrix for non-square matrices
        }

        T det = determinant();
        if (det == 0) {
            return Matrix<T>(0, 0); // Return an empty matrix
        }

        // Create a new matrix for the inverse
        Matrix<T> inverseMatrix(rows, cols);

        // Compute the inverse using the adjugate method
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < cols; ++c) {
                T cofactor = getSubMatrix(r, c).determinant();
                inverseMatrix(c, r) = ((r + c) % 2 == 0 ? 1 : -1) * cofactor / det;
            }
        }

        return inverseMatrix;
    }
    
    // Compute the pseudo inverse of the matrix with the Moore-Penrose algorithm
    // The pseudo inverse of a matrix A is A+ = A^T (A A^T)^-1 
    Matrix<T> pseudoInverse() const {

        Matrix<T> AT = this->transpose();
        Matrix<T> AAT = this->multiply(AT);
        Matrix<T> AATI = AAT.inverse();
        Matrix<T> ATAATI = AT.multiply(AATI);

        return ATAATI;
    }

    // Copy assignment operator
    Matrix& operator=(const Matrix& other) {
        if (this != &other) {
            delete[] data;
            rows = other.rows;
            cols = other.cols;
            data = new T[rows * cols];
            for (size_t i = 0; i < rows * cols; ++i) {
                data[i] = other.data[i];
            }
        }
        return *this;
    }

    // Move assignment operator
    Matrix& operator=(Matrix&& other) {
        if (this != &other) {
            delete[] data;
            data = other.data;
            rows = other.rows;
            cols = other.cols;
            other.data = nullptr; // Invalidate the source
        }
        return *this;
    }
    
    void fillWith(T& elem) {
        for (size_t i = 0; i < rows * cols; ++i) {
            data[i] = elem;
        }
    }

    void fillRow(size_t rowIndex, Vector<T> elems) {
        if (rowIndex < 0 || rowIndex >= rows)
            return;
          
        for (size_t c = 0; c < cols; ++c) {
            (*this)(rowIndex, c) = elems[c];
        }
    }
    
    Vector<T> getRow(size_t rowIndex) const {
        Vector<T> row;

        if (rowIndex < 0 || rowIndex >= rows)
            return row;
        
        for (size_t c = 0; c < cols; ++c) {
            row[c] = (*this)(rowIndex, c);
        }
        return row;
    }
    
    size_t getRows() {
        return rows;
    }
    
    size_t getCols() {
        return cols;
    }
};

#endif
