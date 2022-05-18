#include <iostream>
#include <math.hpp>
#include "interface.h"
#include "test_macros.hpp"

bool test_freq(int freq) {
    D_IN data_def{
        D_IMU{
            matrix::Vector3f(0,0,9.8),
            matrix::Vector3f(0,7.25e-5,0)
        }
    };

    auto iface = NavIface(0,0,0);
    iface.nav()->iter(data_def);
    auto res = iface.nav()->sol().rot();

    if (
        res(0) == 0 &&
        (res(1) - (-1e-8)) < 1e-14 &&
        res(2) == 0
    )
        return true;
    else
        return false;

}

int main() {
    D_IN data_def{
        D_IMU{
            matrix::Vector3f(0,0,9.8),
            matrix::Vector3f(0,7.25e-5,0)
        }
    };
    D_IN data_empty;
    D_IN data_gnss_only_pos{
        D_IMU{
            matrix::Vector3f(0,0,9.8),
            matrix::Vector3f(0,7.25e-5,0)
        },
        D_GNSS{
            matrix::Vector2f(0,-10)
        }
    };

    for (int i = -100; i < 100; i++)
    {
        printf("freq:%d", i);
        TEST(test_freq(i));
    }
    
    auto n = Nav(10);
    n.iter(data_empty);
    auto rot = n.sol().rot();
    for (int i = 0; i<3; i++) {
        TEST(rot(i) == 0);
    }
}