#ifndef VECTOR_HPP
#define VECTOR_HPP

template <typename T>
class Vector {
private:
    T* data;           // Pointer to hold dynamically allocated array
    size_t capacity;   // Total capacity of the vector
    size_t size;       // Number of elements currently in the vector

    // Helper to resize the vector when needed
    void resize(size_t newCapacity) {
        T* newData = new T[newCapacity];  // Allocate new array
        for (size_t i = 0; i < size; ++i) {
            newData[i] = data[i];         // Copy existing elements to new array
        }
        delete[] data;                    // Free old memory
        data = newData;                   // Point to new data
        capacity = newCapacity;           // Update capacity
    }

public:
    // Constructor: initialize vector with a small capacity
    Vector() {
        size = 0;
        capacity = 2;                     // Start with a small capacity
        data = new T[capacity];           // Allocate initial memory
    }

    // Destructor: free allocated memory
    ~Vector() {
        delete[] data;
    }

    // Add a new element to the vector
    void push_back(const T& value) {
        if (size == capacity) {
            resize(capacity * 2);         // Double capacity if full
        }
        data[size++] = value;             // Add the new element
    }

    // Remove the last element from the vector
    void pop_back() {
        if (size > 0) {
            --size;                       // Reduce size but keep memory
        }
    }

    // Access element by index (no bounds checking)
    T& operator[](size_t index) {
        return data[index];
    }

    // Get the number of elements in the vector
    size_t getSize() const {
        return size;
    }

    // Check if the vector is empty
    bool empty() const {
        return size == 0;
    }

    // Clear the vector (reset size but keep capacity)
    void clear() {
        size = 0;
    }
};

#endif // VECTOR_HPP
