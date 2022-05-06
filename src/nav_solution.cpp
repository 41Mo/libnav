#include "nav_solution.h"

const float& Nav_solution::vel(size_t num) { return velocity(num); }

const matrix::Vector3f& Nav_solution::vel() { return velocity; }

void Nav_solution::vel(float vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->velocity(i);
  }
}

const float& Nav_solution::vel_sns(size_t num) { return velocity_sns(num); }

const matrix::Vector3f& Nav_solution::vel_sns() { return velocity_sns; }

void Nav_solution::vel_sns(float vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->velocity_sns(i);
  }
}

const float& Nav_solution::rot(size_t num) { return rotation(num); }

const matrix::Eulerf& Nav_solution::rot() { return rotation; }

void Nav_solution::rot(float vec_in[3]) {
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = this->rotation(i);
  }
}

const float& Nav_solution::pos(size_t num) { return position(num); }

const matrix::Vector2f& Nav_solution::pos() { return position; }

void Nav_solution::pos(float vec_in[2]) {
  for (size_t i = 0; i < 2; i++) {
    vec_in[i] = position(i);
  }
}

const float& Nav_solution::pos_sns(size_t num) { return position_sns(num); }

const matrix::Vector2f& Nav_solution::pos_sns() { return position_sns; }

void Nav_solution::pos_sns(float vec_in[2]) {
  for (size_t i = 0; i < 2; i++) {
    vec_in[i] = position_sns(i);
  }
}
