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

    Vector() : Vector(2) {}

    Vector(size_t _capacity) {
        size = 0;
        capacity = _capacity;             // Start with a small capacity
        data = new T[capacity];           // Allocate initial memory
    }

    Vector(const Vector& other) : size(other.size), capacity(other.capacity) {
        data = new T[capacity];
        for (size_t i = 0; i < size; ++i) {
            data[i] = other.data[i];
        }
    }

    Vector(Vector&& other) noexcept : data(other.data), size(other.size), capacity(other.capacity) {
        /**
         * Move constructor
         */
        other.data = nullptr;  // Transfer ownership
        other.size = 0;
        other.capacity = 0;
    }

    Vector& operator=(Vector&& other) noexcept {
        /**
         * Move assignment operator
         */
        if (this != &other) {
            delete[] data; // Clean up current resources

            // Transfer ownership
            data = other.data;
            size = other.size;
            capacity = other.capacity;

            other.data = nullptr;
            other.size = 0;
            other.capacity = 0;
        }
        return *this;
    }

    Vector& operator=(const Vector& other) {

        if (this == &other) 
            return *this;
        
        delete[] data;
        size = other.size;
        capacity = other.capacity;  
        data = new T[capacity];
        
        for (size_t i = 0; i < size; ++i) {
            data[i] = other.data[i];
        }
        
        return *this;
    }

    // Destructor: free allocated memory
    ~Vector() {
        delete[] data;
    }

    // Add a new element to the vector
    void pushBack(const T& value) {
        if (size == capacity) {
            resize(capacity * 2);         // Double capacity if full
        }
        data[size++] = value;             // Add the new element
    }

    // Remove the last element from the vector
    void popBack() {
        if (size > 0) {
            --size;                       // Reduce size but keep memory
        }
    }

    void fill(T elem) {
      for (size_t i = 0; i < size; ++i) {
          data[i] = elem;
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
