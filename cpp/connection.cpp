#include <iostream>
#include <vector>
#include <stdio.h>
#include <cstring>

#include "connection.h"

void Connection::send(std::vector<unsigned char>& data, float& q1, float& q2, float& q3) {
    data.clear();
    unsigned char c[3 * sizeof(float)];
    std::memcpy(&c[0], &q1, sizeof(float));
    std::memcpy(&c[4], &q2, sizeof(float));
    std::memcpy(&c[8], &q3, sizeof(float));
    data.insert(data.begin(), std::begin(c), std::end(c));
}

void Connection::receive(const std::vector<unsigned char>& data, float& q1, float& q2, float& q3) {
    q1 = *((float*)&data[0]);
    q2 = *((float*)&data[4]);
    q3 = *((float*)&data[8]);
}
