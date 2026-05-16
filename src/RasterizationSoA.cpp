#include "RasterizationSoA.h"
#include "Particle.h"
#include <queue>
#include <vector>

double RasterizationSoA::findHeightValByScanline(int index, ClothSoA& cloth) {
    int xpos = cloth.getX(index);
    int ypos = cloth.getY(index);

    for (int i = xpos + 1; i < cloth.num_particles_width; i++) {
        double height = cloth.nearest_height[cloth.getIndex(i, ypos)];
        if (height > MIN_INF)
            return height;
    }

    for (int i = xpos - 1; i >= 0; i--) {
        double height = cloth.nearest_height[cloth.getIndex(i, ypos)];
        if (height > MIN_INF)
            return height;
    }

    for (int j = ypos - 1; j >= 0; j--) {
        double height = cloth.nearest_height[cloth.getIndex(xpos, j)];
        if (height > MIN_INF)
            return height;
    }

    for (int j = ypos + 1; j < cloth.num_particles_height; j++) {
        double height = cloth.nearest_height[cloth.getIndex(xpos, j)];
        if (height > MIN_INF)
            return height;
    }

    return findHeightValByNeighbor(index, cloth);
}

double RasterizationSoA::findHeightValByNeighbor(int index, ClothSoA& cloth) {
    std::queue<int> nqueue;
    std::vector<int> touched;

    cloth.visited[index] = 1;
    touched.push_back(index);

    const int *begin = cloth.neighborsBegin(index);
    const int *end = cloth.neighborsEnd(index);
    for (const int *it = begin; it != end; ++it) {
        int neighbor = *it;
        if (!cloth.visited[neighbor]) {
            cloth.visited[neighbor] = 1;
            touched.push_back(neighbor);
            nqueue.push(neighbor);
        }
    }

    while (!nqueue.empty()) {
        int neighbor = nqueue.front();
        nqueue.pop();

        if (cloth.nearest_height[neighbor] > MIN_INF) {
            for (std::size_t i = 0; i < touched.size(); i++)
                cloth.visited[touched[i]] = 0;
            return cloth.nearest_height[neighbor];
        }

        begin = cloth.neighborsBegin(neighbor);
        end = cloth.neighborsEnd(neighbor);
        for (const int *it = begin; it != end; ++it) {
            int next = *it;
            if (!cloth.visited[next]) {
                cloth.visited[next] = 1;
                touched.push_back(next);
                nqueue.push(next);
            }
        }
    }

    for (std::size_t i = 0; i < touched.size(); i++)
        cloth.visited[touched[i]] = 0;
    return MIN_INF;
}

void RasterizationSoA::RasterTerrain(ClothSoA& cloth, csf::PointCloud& pc) {
    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int col = int(deltaX / cloth.step_x + 0.5);
        int row = int(deltaZ / cloth.step_y + 0.5);

        if ((col >= 0) && (row >= 0) &&
            (col < cloth.num_particles_width) && (row < cloth.num_particles_height)) {
            int idx = cloth.getIndex(col, row);
            double pc2particleDist = SQUARE_DIST_SOA(
                pc_x, pc_z,
                cloth.posX(idx),
                cloth.posZ(idx)
            );

            if (pc2particleDist < cloth.tmp_dist[idx]) {
                cloth.tmp_dist[idx] = pc2particleDist;
                cloth.nearest_height[idx] = pc[i].y;
                cloth.nearest_idx[idx] = i;
            }
        }
    }

    int particleCount = cloth.getSize();
    cloth.heightvals.resize(particleCount);
    for (int i = 0; i < particleCount; i++) {
        double nearestHeight = cloth.nearest_height[i];
        if (nearestHeight > MIN_INF) {
            cloth.heightvals[i] = nearestHeight;
        } else {
            cloth.heightvals[i] = findHeightValByScanline(i, cloth);
        }
    }
}
