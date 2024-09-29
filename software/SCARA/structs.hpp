#ifndef POINT_HPP
#define POINT_HPP

// Structure representing a point in space
struct Point {
    float x, y, z;

    // Default constructor
    Point() : x(0), y(0), z(0) {}

    // Constructor to initialize the Point
    Point(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    // Overload the - operator to subtract two Points, resulting in a Vector3D
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y, z - other.z);
    }

    double magnitude() const {
      return sqrt(x * x + y * y + z * z);
    }
};

// Structure to hold the inverse kinematics solution (angles for each joint)
struct IKSolution {
    float q1, q2, q3;  // Joint angles

    // Default constructor
    IKSolution() : q1(0), q2(0), q3(0) {}

    // Constructor to initialize the Point
    IKSolution(float _q1, float _q2, float _q3) : q1(_q1), q2(_q2), q3(_q3) {}
};

// Structure to hold the steps for each motor
struct Steps {

    // On the arduino I'm using, an int is only 2 bytes. A 16-bit signed integer 
    // has a range of values from -32,768 to 32,767, meaning that any value 
    // larger than 32,767 will overflow. This led to many problems. A long
    // has a wider range.
    long s1, s2, s3;  // Steps

    // Default constructor
    Steps() : s1(0), s2(0), s3(0) {}

    // Constructor to initialize the Point
    Steps(long _s1, long _s2, long _s3) : s1(_s1), s2(_s2), s3(_s3) {}
};

#endif