#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "config.hpp"

bool isReachable(float x, float y, float z) {
  /**
   * Check if a cartesian point is reachable by the robot.
   */

  float distance = sqrt(x * x + y * y);
  return distance <= (L1 + L2) && JOINT_1_MIN_LIMIT <= z && z <= JOINT_1_MAX_LIMIT;
}

bool inverseKinematics(float x, float y, float z, float &q1, float &q2, float &q3) {
  /**
    * Compute the inverse kinematics for the SCARA arm. The result is put in the q1, q2 and q3
    * variables. The method returns true if the point is reachable and thus it exists an
    * inverse kinematics solution, false otherwise.
    */

  if (!isReachable(x, y, z)) {
    return false;  // The point is out of reach
  }

  // First joint (directly determined by z)
  q1 = z;

  // Third joint (calculating c3 and s3)
  float c3 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float s3_p = sqrt(1 - c3 * c3);
  float q3_p = atan2(s3_p, c3);

  // Second joint (calculating q2 for the positive s3 solution)
  float A = L2 * c3 + L1;
  float B_p = L2 * s3_p;
  float q2_p = atan2(y, x) - atan2(B_p, A);
  
  /*
  float c2_p = (A * x + B_p * y);
  float s2_p = (-B_p * y - A * x);
  float q2_p = atan2(s2_p, c2_p);
  */

  // Assign the solutions
  q2 = q2_p;
  q3 = q3_p;

  return true;
}

#endif // KINEMATICS_HPP