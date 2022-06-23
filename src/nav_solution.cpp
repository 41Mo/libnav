#include "nav_solution.h"

namespace NavA {
const double& Nav_solution::vel(size_t num) { return velocity(num); }

const matrix::Vector3d& Nav_solution::vel() { return velocity; }

void Nav_solution::vel(double vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->velocity(i);
  }
}

const double& Nav_solution::rot(size_t num) { return rotation(num); }

const matrix::Eulerd& Nav_solution::rot() { return rotation; }

void Nav_solution::rot(double vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->rotation(i);
  }
}

const double& Nav_solution::pos(size_t num) { return position(num); }

const matrix::Vector2d& Nav_solution::pos() { return position; }

void Nav_solution::pos(double vec_in[2]) {
  for (size_t i = 0; i < 2; i++) {
    vec_in[i] = position(i);
  }
}
}
