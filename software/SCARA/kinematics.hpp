#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "config.hpp"
#include "vector.hpp"
#include "structs.hpp"

bool isReachable(const Point& p) {
  /**
   * Check if a cartesian point is reachable by the robot.
   */

  float distance = sqrt(p.x * p.x + p.y * p.y);
  return distance <= (L1 + L2) && JOINT_1_MIN_LIMIT <= p.z && p.z <= JOINT_1_MAX_LIMIT;
}

bool inverseKinematics(const Point& p, IKSolution& solution) {
  /**
    * Compute the inverse kinematics for the SCARA arm. The result is put in the q1, q2 and q3
    * variables. The method returns true if the point is reachable and thus it exists an
    * inverse kinematics solution, false otherwise.
    */

  if (!isReachable(p)) {
    return false;  // The point is out of reach
  }

  // First joint (directly determined by z)
  solution.q1 = p.z;

  // Third joint (calculating c3 and s3)
  float c3 = (p.x * p.x + p.y * p.y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float s3_p = sqrt(1 - c3 * c3);
  float q3_p = atan2(s3_p, c3);

  // Second joint (calculating q2 for the positive s3 solution)
  float A = L2 * c3 + L1;
  float B_p = L2 * s3_p;
  float q2_p = atan2(p.y, p.x) - atan2(B_p, A);

  /*
  float c2_p = (A * x + B_p * y);
  float s2_p = (-B_p * y - A * x);
  float q2_p = atan2(s2_p, c2_p);
  */

  // Assign the solutions
  solution.q2 = q2_p;
  solution.q3 = q3_p;

  // The second joint angle should be within the [-pi/2, +pi/2] range
  if (solution.q2 < JOINT_2_MIN_LIMIT || solution.q2 > JOINT_2_MAX_LIMIT)
    return false;

  // The third joint angle should be within the [-8pi/9, +8pi/9] range
  if (solution.q3 < JOINT_3_MIN_LIMIT || solution.q3 > JOINT_3_MAX_LIMIT)
    return false;

  return true;
}

Vector<IKSolution> inverseKinematicsVector(const Vector<Point>& points) {
    /**
     * Compute the inverse kinematics solutions for each point in the array. If a single
     * point in input is not reachable, the entire trajectory is not feasible and the 
     * method will return an empty vector.
     */    

    Vector<IKSolution> solutions;

    for (size_t i = 0; i < points.getSize(); i++) {
        IKSolution solution;
        if (!inverseKinematics(points[i], solution)) {
            // If any point is not reachable, return an empty vector
            solutions.clear();
            return solutions;
        }
        solutions.push_back(solution);
    }

    return solutions;
}

#endif  // KINEMATICS_HPP