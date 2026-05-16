#ifndef _CLOTH_SOA_H_
#define _CLOTH_SOA_H_

#include "Cloth.h"
#include "Vec3.h"
#include <cstddef>
#include <vector>

class ClothSoA {
private:
    int constraint_iterations;
    double time_step2;

    std::vector<int> neighbor_begin;
    std::vector<int> neighbor_indices;

    std::vector<int> neighbor_write_offsets;
    std::vector<double> pos_y_snapshot;
    std::vector<double> delta_y;

    void countConstraint(int p1, int p2);
    void writeConstraint(int p1, int p2);
    void buildNeighbors();

public:
    Vec3 origin_pos;
    double step_x;
    double step_y;
    int num_particles_width;
    int num_particles_height;

    std::vector<double> pos_y;
    std::vector<double> old_y;
    std::vector<double> acceleration_y;
    std::vector<unsigned char> movable;
    std::vector<unsigned char> visited;

    std::vector<double> heightvals;
    std::vector<double> nearest_height;
    std::vector<double> tmp_dist;
    std::vector<std::size_t> nearest_idx;

    ClothSoA(const Vec3& origin_pos,
             int         num_particles_width,
             int         num_particles_height,
             double      step_x,
             double      step_y,
             int         rigidness,
             double      time_step);

    int getSize() const;
    int getIndex(int x, int y) const;
    int getX(int index) const;
    int getY(int index) const;
    double posX(int index) const;
    double posZ(int index) const;
    double posY(int index) const;
    double posYAt(int x, int y) const;

    const int* neighborsBegin(int index) const;
    const int* neighborsEnd(int index) const;
    const std::vector<int>& neighborBeginVector() const;
    const std::vector<int>& neighborIndexVector() const;

    void addForceY(double force_y);
    double timeStep(ClothStepProfile *profile = 0);
    void terrCollision();
    std::vector<double> toVector() const;
};

#endif
