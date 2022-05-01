#include "nav_solution.h"

const float& Nav_solution::vel(size_t num) { return v(num); }

const matrix::Vector3f& Nav_solution::vel() { return v; }

void Nav_solution::vel(float vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->v(i);
  }
}

const float& Nav_solution::rot(size_t num) { return r(num); }

const matrix::Eulerf& Nav_solution::rot() { return r; }

void Nav_solution::rot(float vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->r(i);
  }
}

const float& Nav_solution::pos(size_t num) { return p(num); }

const matrix::Vector2f& Nav_solution::pos() { return p; }

void Nav_solution::pos(float vec_in[2]) {
  for (size_t i = 0; i < 2; i++) {
    vec_in[i] = p(i);
  }
}