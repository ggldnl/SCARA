#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "config.hpp"
#include "vector.hpp"
#include "matrix.hpp"
#include "structs.hpp"

bool isReachable(const Point& p) {
  float distance = sqrt(p.x * p.x + p.y * p.y);
  return distance <= (L1 + L2) && JOINT_1_MIN_LIMIT <= p.z && p.z <= JOINT_1_MAX_LIMIT;
}

bool inverseKinematics(const Point& p, IKSolution& solution) {
  if (!isReachable(p)) {
    return false;  // The point is out of reach
  }

  solution.q1 = p.z;
  float c3 = (p.x * p.x + p.y * p.y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float s3_p = sqrt(1 - c3 * c3);
  float q3_p = atan2(s3_p, c3);

  float A = L2 * c3 + L1;
  float B_p = L2 * s3_p;
  float q2_p = atan2(p.y, p.x) - atan2(B_p, A);

  solution.q2 = q2_p;
  solution.q3 = q3_p;

  if (solution.q2 < JOINT_2_MIN_LIMIT || solution.q2 > JOINT_2_MAX_LIMIT)
    return false;

  if (solution.q3 < JOINT_3_MIN_LIMIT || solution.q3 > JOINT_3_MAX_LIMIT)
    return false;

  return true;
}

Vector<IKSolution> inverseKinematicsVector(const Vector<Point>& points) {
  Vector<IKSolution> solutions;

  for (size_t i = 0; i < points.getSize(); i++) {
    IKSolution solution;
    if (!inverseKinematics(points[i], solution)) {
      solutions.clear();
      return solutions;
    }
    solutions.push_back(solution);
  }

  return solutions;
}

const Matrix<float>& getJacobian(const IKSolution& joints) {
  static Matrix<float> jacobian(3, 3);

  float q2 = joints.q2;
  float q3 = joints.q3;

  jacobian(0, 0) = 0;
  jacobian(0, 1) = -L2 * sin(q2 + q3) - L1 * sin(q2);
  jacobian(0, 2) = -L2 * sin(q2 + q3);

  jacobian(1, 0) = 0;
  jacobian(1, 1) = L2 * cos(q2 + q3) + L1 * cos(q2);
  jacobian(1, 2) = L2 * cos(q2 + q3);

  jacobian(2, 0) = 0;
  jacobian(2, 1) = 0;
  jacobian(2, 2) = 1;

  return jacobian;
}

const Matrix<float>& getJacobianInverse(const IKSolution& joints, const float epsilon = 1e-6) {
  static Matrix<float> jacobianInverse(3, 3);

  float q2 = joints.q2;
  float q3 = joints.q3;

  float denom_1 = L1 * cos(q2 + q3) * sin(q2) - L1 * sin(q2 + q3) * cos(q2);
  float denom_2 = L1 * L2 * cos(q2 + q3) * sin(q2) - L1 * L2 * sin(q2 + q3) * cos(q2);

  if (fabs(denom_1) < epsilon) denom_1 = epsilon;
  if (fabs(denom_2) < epsilon) denom_2 = epsilon;

  jacobianInverse(0, 0) = 0;
  jacobianInverse(0, 1) = 0;
  jacobianInverse(0, 2) = 1;

  jacobianInverse(1, 0) = -cos(q2 + q3) / denom_1;
  jacobianInverse(1, 1) = -sin(q2 + q3) / denom_1;
  jacobianInverse(1, 2) = 0;

  jacobianInverse(2, 0) = (L2 * cos(q2 + q3) + L1 * cos(q2)) / denom_2;
  jacobianInverse(2, 1) = (L2 * sin(q2 + q3) + L1 * sin(q2)) / denom_2;
  jacobianInverse(2, 2) = 0;

  return jacobianInverse;
}

void perturbJointAngles(IKSolution& joints, const float epsilon = 1e-6, const float perturbation = 1e-2) {
    /**
     * Slightly modify the joint angles to move out of a singularity.
     * Apply a small perturbation to one or more joints to "nudge" out of singular configurations.
     */

    float q1 = joints.q1;
    float q2 = joints.q2;
    float q3 = joints.q3;
    
    // Apply a small perturbation to q2 and q3
    joints.q1 = q1;  // Keep q1 the same as we cannot have singularities here
    joints.q2 = (fabs(q2) < epsilon || fabs(q2 - M_PI/2) < epsilon || fabs(q2 + M_PI/2) < epsilon) ? q2 + perturbation : q2;
    joints.q3 = (fabs(q3) < epsilon || fabs(q3 - M_PI/2) < epsilon || fabs(q3 + M_PI/2) < epsilon) ? q3 + perturbation : q3;

}

Vector<float> computeJointVelocities(IKSolution& currentJointConfiguration, const Vector<float>& velocityVector, const float epsilon = 1e-6) {

  const Matrix<float>& jacobian = getJacobian(currentJointConfiguration);

  float det = jacobian(0, 0) * (jacobian(1, 1) * jacobian(2, 2) - jacobian(1, 2) * jacobian(2, 1)) -
              jacobian(0, 1) * (jacobian(1, 0) * jacobian(2, 2) - jacobian(1, 2) * jacobian(2, 0)) +
              jacobian(0, 2) * (jacobian(1, 0) * jacobian(2, 1) - jacobian(1, 1) * jacobian(2, 0));

  // This is what I originally tried but it went really bad
  // const Matrix<float>& jacobianInverse = (fabs(det) > epsilon) ? getJacobianInverse(currentJointConfiguration) : getJacobianPseudoInverse(jacobian);

  // Instead of computing the pseudo inverse of a matrix on an arduino (mh...) we can slightly change the current joint configuration such that we 
  // are no more in a singularity. These computations will be rounded down to the delays between steps, I don't think we should worry about precision.
  if (fabs(det) < epsilon)
      perturbJointAngles(currentJointConfiguration);

  const Matrix<float>& jacobianInverse = getJacobianInverse(currentJointConfiguration);

  Vector<float> jointVelocities = jacobianInverse.multiply(velocityVector);

  /*
  Logger::debug("Joint configuration:");
  Logger::debug("{}\t {}\t {}", currentJointConfiguration.q1, currentJointConfiguration.q2, currentJointConfiguration.q3);

  Logger::debug("Jacobian Inverse:");
  Logger::debug("{}\t {}\t {}", jacobianInverse(0, 0), jacobianInverse(0, 1), jacobianInverse(0, 2));
  Logger::debug("{}\t {}\t {}", jacobianInverse(1, 0), jacobianInverse(1, 1), jacobianInverse(1, 2));
  Logger::debug("{}\t {}\t {}", jacobianInverse(2, 0), jacobianInverse(2, 1), jacobianInverse(2, 2));
  
  Logger::debug("Joint velocities:");
  Logger::debug("{}\t {}\t {}", jointVelocities[0], jointVelocities[1], jointVelocities[2]);
  */

  return jointVelocities;
}

#endif  // KINEMATICS_HPP
