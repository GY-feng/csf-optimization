#include "ClothSoA.h"
#include "Particle.h"
#include <chrono>
#include <cmath>

namespace {
using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point& start, const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}
}

ClothSoA::ClothSoA(const Vec3& _origin_pos,
                   int         _num_particles_width,
                   int         _num_particles_height,
                   double      _step_x,
                   double      _step_y,
                   int         rigidness,
                   double      time_step)
    : constraint_iterations(rigidness),
      time_step2(time_step * time_step),
      origin_pos(_origin_pos),
      step_x(_step_x),
      step_y(_step_y),
      num_particles_width(_num_particles_width),
      num_particles_height(_num_particles_height) {

    int particleCount = getSize();
    pos_y.assign(particleCount, origin_pos.f[1]);
    old_y.assign(particleCount, origin_pos.f[1]);
    acceleration_y.assign(particleCount, 0.0);
    movable.assign(particleCount, 1);
    visited.assign(particleCount, 0);
    heightvals.assign(particleCount, MIN_INF);
    nearest_height.assign(particleCount, MIN_INF);
    tmp_dist.assign(particleCount, MAX_INF);
    nearest_idx.assign(particleCount, 0);
    pos_y_snapshot.assign(particleCount, origin_pos.f[1]);
    delta_y.assign(particleCount, 0.0);

    buildNeighbors();
}

int ClothSoA::getSize() const {
    return num_particles_width * num_particles_height;
}

int ClothSoA::getIndex(int x, int y) const {
    return y * num_particles_width + x;
}

int ClothSoA::getX(int index) const {
    return index % num_particles_width;
}

int ClothSoA::getY(int index) const {
    return index / num_particles_width;
}

double ClothSoA::posX(int index) const {
    return origin_pos.f[0] + getX(index) * step_x;
}

double ClothSoA::posZ(int index) const {
    return origin_pos.f[2] + getY(index) * step_y;
}

double ClothSoA::posY(int index) const {
    return pos_y[index];
}

double ClothSoA::posYAt(int x, int y) const {
    return pos_y[getIndex(x, y)];
}

const int* ClothSoA::neighborsBegin(int index) const {
    return neighbor_indices.data() + neighbor_begin[index];
}

const int* ClothSoA::neighborsEnd(int index) const {
    return neighbor_indices.data() + neighbor_begin[index + 1];
}

const std::vector<int>& ClothSoA::neighborBeginVector() const {
    return neighbor_begin;
}

const std::vector<int>& ClothSoA::neighborIndexVector() const {
    return neighbor_indices;
}

void ClothSoA::countConstraint(int p1, int p2) {
    neighbor_begin[p1 + 1]++;
    neighbor_begin[p2 + 1]++;
}

void ClothSoA::writeConstraint(int p1, int p2) {
    neighbor_indices[neighbor_write_offsets[p1]++] = p2;
    neighbor_indices[neighbor_write_offsets[p2]++] = p1;
}

void ClothSoA::buildNeighbors() {
    int particleCount = getSize();
    neighbor_begin.assign(static_cast<std::size_t>(particleCount) + 1, 0);

    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            if (x < num_particles_width - 1)
                countConstraint(getIndex(x, y), getIndex(x + 1, y));

            if (y < num_particles_height - 1)
                countConstraint(getIndex(x, y), getIndex(x, y + 1));

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                countConstraint(getIndex(x, y), getIndex(x + 1, y + 1));

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                countConstraint(getIndex(x + 1, y), getIndex(x, y + 1));
        }
    }

    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            if (x < num_particles_width - 2)
                countConstraint(getIndex(x, y), getIndex(x + 2, y));

            if (y < num_particles_height - 2)
                countConstraint(getIndex(x, y), getIndex(x, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                countConstraint(getIndex(x, y), getIndex(x + 2, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                countConstraint(getIndex(x + 2, y), getIndex(x, y + 2));
        }
    }

    for (int i = 1; i <= particleCount; i++) {
        neighbor_begin[i] += neighbor_begin[i - 1];
    }

    neighbor_indices.assign(neighbor_begin[particleCount], 0);
    neighbor_write_offsets.assign(neighbor_begin.begin(), neighbor_begin.end() - 1);

    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            if (x < num_particles_width - 1)
                writeConstraint(getIndex(x, y), getIndex(x + 1, y));

            if (y < num_particles_height - 1)
                writeConstraint(getIndex(x, y), getIndex(x, y + 1));

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                writeConstraint(getIndex(x, y), getIndex(x + 1, y + 1));

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                writeConstraint(getIndex(x + 1, y), getIndex(x, y + 1));
        }
    }

    for (int x = 0; x < num_particles_width; x++) {
        for (int y = 0; y < num_particles_height; y++) {
            if (x < num_particles_width - 2)
                writeConstraint(getIndex(x, y), getIndex(x + 2, y));

            if (y < num_particles_height - 2)
                writeConstraint(getIndex(x, y), getIndex(x, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                writeConstraint(getIndex(x, y), getIndex(x + 2, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                writeConstraint(getIndex(x + 2, y), getIndex(x, y + 2));
        }
    }

    neighbor_write_offsets.clear();
    neighbor_write_offsets.shrink_to_fit();
}

void ClothSoA::addForceY(double force_y) {
    int particleCount = getSize();
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++) {
        acceleration_y[i] += force_y;
    }
}

double ClothSoA::timeStep(ClothStepProfile *profile) {
    int particleCount = getSize();

    Clock::time_point stageStart = Clock::now();
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++) {
        if (movable[i]) {
            double temp = pos_y[i];
            pos_y[i] = pos_y[i] + (pos_y[i] - old_y[i]) * (1.0 - DAMPING) + acceleration_y[i] * time_step2;
            old_y[i] = temp;
        }
    }
    if (profile) {
        profile->verlet_ms += elapsedMs(stageStart, Clock::now());
    }

    stageStart = Clock::now();
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++) {
        pos_y_snapshot[i] = pos_y[i];
        delta_y[i] = 0.0;
    }

#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int p1 = 0; p1 < particleCount; p1++) {
        if (!movable[p1]) {
            continue;
        }

        double localDelta = 0.0;
        const int *begin = neighborsBegin(p1);
        const int *end = neighborsEnd(p1);
        for (const int *it = begin; it != end; ++it) {
            int p2 = *it;
            double correction = pos_y_snapshot[p2] - pos_y_snapshot[p1];

            if (movable[p2]) {
                double correctionHalf = correction * (constraint_iterations > 14 ? 0.5 : doubleMove1[constraint_iterations]);
                localDelta += correctionHalf;
            } else {
                double correctionHalf = correction * (constraint_iterations > 14 ? 1 : singleMove1[constraint_iterations]);
                localDelta += correctionHalf;
            }
        }
        delta_y[p1] = localDelta;
    }

#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++) {
        if (movable[i]) {
            pos_y[i] += delta_y[i];
        }
    }
    if (profile) {
        profile->constraint_ms += elapsedMs(stageStart, Clock::now());
    }

    stageStart = Clock::now();
    double maxDiff = 0;
#ifdef CSF_USE_OPENMP
#pragma omp parallel for reduction(max:maxDiff)
#endif
    for (int i = 0; i < particleCount; i++) {
        if (movable[i]) {
            double diff = std::fabs(old_y[i] - pos_y[i]);
            if (diff > maxDiff)
                maxDiff = diff;
        }
    }
    if (profile) {
        profile->maxdiff_ms += elapsedMs(stageStart, Clock::now());
    }

    return maxDiff;
}

void ClothSoA::terrCollision() {
    int particleCount = getSize();
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++) {
        if (pos_y[i] < heightvals[i]) {
            if (movable[i]) {
                pos_y[i] += heightvals[i] - pos_y[i];
            }
            movable[i] = 0;
        }
    }
}

std::vector<double> ClothSoA::toVector() const {
    std::vector<double> clothCoordinates;
    int particleCount = getSize();
    clothCoordinates.reserve(static_cast<std::size_t>(particleCount) * 3);
    for (int i = 0; i < particleCount; i++) {
        clothCoordinates.push_back(posX(i));
        clothCoordinates.push_back(posZ(i));
        clothCoordinates.push_back(-pos_y[i]);
    }
    return clothCoordinates;
}
