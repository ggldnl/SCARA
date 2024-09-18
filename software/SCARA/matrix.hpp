#ifndef MATRIX_HPP
#define MATRIX_HPP

#include "vector.hpp"

template <typename T>
class Matrix {
private:
    Vector<Vector<T>> data; // 2D vector to store matrix data
    size_t rows, cols;

public:
    // Constructor to initialize the matrix with rows and columns
    Matrix(size_t rows, size_t cols) : rows(rows), cols(cols), data(rows) {
        for (size_t i = 0; i < rows; ++i) {
            Vector<T> row(cols);
            for (size_t j = 0; j < cols; ++j) {
                row[j] = 0; // Initialize with zeros
            }
            data[i] = row;
        }
    }

    T& operator()(size_t row, size_t col) const {
        return data[row][col];
    }

    size_t numRows() const {
        return rows;
    }

    size_t numCols() const {
        return cols;
    }

    Vector<T> multiply(const Vector<T>& vec) const {
        /**
         * Matrix multiplication with a vector
         */

        Vector<T> result(rows); // Note: result should have the size of rows, not cols

        if (cols != vec.getSize()) {
            return;
        }

        for (size_t i = 0; i < rows; ++i) {
            T sum = 0;
            for (size_t j = 0; j < cols; ++j) {
                sum += data[i][j] * vec[j];
            }
            result[i] = sum;
        }

        return result;
    }

    Matrix<T> multiply(const Matrix<T>& other) const {
        /**
         * Matrix multiplication with another matrix
         */

        if (cols != other.rows) {
            return;
        }

        Matrix<T> result(rows, other.cols); 

        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < other.cols; ++j) {
                for (size_t k = 0; k < cols; ++k) {
                    result(i, j) += data[i][k] * other(k, j);
                }
            }
        }

        return result;
    }

    Matrix<T> transpose() const {
        /**
         * Returns the transpose of the matrix
         */

        Matrix<T> result(cols, rows);

        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result(j, i) = data[i][j];
            }
        }

        return result;
    }
};

#endif // MATRIX_HPP
