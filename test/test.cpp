#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>

#include "constants.h"
#include "analysis_api.h"

std::vector<std::vector<std::string>> parse_csv(std::string fn) {
    std::string filename{fn};
    std::ifstream input{filename};

    std::vector<std::vector<std::string>> csvRows;

    if (!input.is_open()) {
      std::cerr << "Couldn't read file: " << filename << "\n";
      return csvRows;
    }


    for (std::string line; std::getline(input, line);) {
      std::istringstream ss(std::move(line));
      std::vector<std::string> row;
      if (!csvRows.empty()) {
         // We expect each row to be as big as the first row
        row.reserve(csvRows.front().size());
      }
      // std::getline can split on other characters, here we use ','
      for (std::string value; std::getline(ss, value, ';');) {
        row.push_back(std::move(value));
      }
      csvRows.push_back(std::move(row));
    }

    return csvRows;
}

void print_value(float v) {
    std::cout << std::setw(15) << v;
}
void test_print(vec_body acc, vec_body gyr) {
    print_value(acc.X);
    print_value(acc.Y);
    print_value(acc.Z);
    print_value(gyr.X);
    print_value(gyr.Y);
    print_value(gyr.Z);
    std::cout << "\n";
}

float precise_float(std::string s) {
    // dirty hack
    std::ostringstream out;
    out << std::setprecision(7) << std::stof(s);
    float percise = std::stof(out.str());
    return percise;
}

int main(int argc, char const *argv[])
{
    /*
    auto csv = parse_csv("test/test_data.csv");
    int size = csv.size();
    vec_body *acc = new vec_body[size];
    vec_body *gyr = new vec_body[size];

    // starting from 1 to skip header
    int header = 1;
    for (auto i = header; i<size; i++) {
        auto row = csv.at(i);
        acc[i-1].X = precise_float(row.at(2));
        acc[i-1].Y = precise_float(row.at(3));
        acc[i-1].Z = precise_float(row.at(4));
        gyr[i-1].X = precise_float(row.at(5));
        gyr[i-1].Y = precise_float(row.at(6));
        gyr[i-1].Z = precise_float(row.at(7));
        //test_print(acc[i], gyr[i]);
    }
    Nav *api = Analysis_api_new();
    SENSORS s{size, acc, gyr};
    */

    auto frequency = 100;
    auto time = 60;
    auto points = time*frequency;

    vec_body *acc = new vec_body[points];
    vec_body *gyr = new vec_body[points];
    for (auto i =0; i< points; i++) {
      acc[i].X = 0;
      acc[i].Y = 0;
      acc[i].Z = 9.81;
      gyr[i].X = 0;
      gyr[i].Y = U;
      gyr[i].Z = 0;
    }

    Nav *api = Analysis_api_new();
    SENS_IN s{points, acc, gyr};
    api_init(api, 0,0, frequency,time);
    api_set_sens(api, s);
    api_loop(api);
    auto G = api_get_g();
    printf("%f\n", G);
    NAV_OUT a = api_get_data(api);
    for (auto i = 0; i< s.size; i++) {
      auto k = a.pitch[i];
      printf("%1.21f\n", k);
    }
    return 0;
}
