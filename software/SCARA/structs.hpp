#ifndef POINT_HPP
#define POINT_HPP

// Structure representing a point in space
struct Point {
    float x, y, z;

    // Default constructor
    Point() : x(0), y(0), z(0) {}

    // Constructor to initialize the Point
    Point(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
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
    int s1, s2, s3;  // Steps
};

#endif