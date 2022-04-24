#include "nav_solution.h"

const float& Nav_solution::vel(int num) { return (*vel_ptr)(num); }

const matrix::Vector3f& Nav_solution::vel() { return *(vel_ptr); }

void Nav_solution::vel(float vec_in[3]) {
  auto v = *(vel_ptr);
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = v(i);
  }
}

const float& Nav_solution::rot(int num) { return (*rot_ptr)(num); }

const matrix::Eulerf& Nav_solution::rot() { return *(rot_ptr); }

void Nav_solution::rot(float vec_in[3]) {
  auto r = *(rot_ptr);
  for (size_t i = 0; i < 3; i++) {
    vec_in[i] = r(i);
  }
}

const float& Nav_solution::pos(int num) { return (*coord_ptr)(num); }

const matrix::Vector2f& Nav_solution::pos() { return *(coord_ptr); }

void Nav_solution::pos(float vec_in[2]) {
  auto r = *(rot_ptr);
  for (size_t i = 0; i < 2; i++) {
    vec_in[i] = r(i);
  }
}

void Nav_solution::setup(matrix::Vector3f* vel, matrix::Eulerf* rpy,
                         matrix::Vector2f* coord) {
  vel_ptr = vel;
  rot_ptr = rpy;
  coord_ptr = coord;
}