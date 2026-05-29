#ifndef _FEC_H_Included_
#define _FEC_H_Included_
#include "../include/utils/FEC.h"
#endif

// Union-Find with path compression and union by rank
struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;
    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; i++) parent[i] = i;
    }
    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int x, int y) {
        int rx = find(x), ry = find(y);
        if (rx == ry) return;
        if (rank[rx] < rank[ry]) std::swap(rx, ry);
        parent[ry] = rx;
        if (rank[rx] == rank[ry]) rank[rx]++;
    }
};

// compare the nuber tag of two points
bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1)
{
    return p0.nNumberTag < p1.nNumberTag;
}

std::vector<pcl::PointIndices> FEC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_component_size, double tolerance, int max_n) {

    unsigned long i, j;
    if (cloud->size() < min_component_size)
    {
        PCL_ERROR("Could not find any cluster");
        return {};
    }
    // KD treee, and the input data
    pcl::KdTreeFLANN<pcl::PointXYZ> cloud_kdtreeflann;
    cloud_kdtreeflann.setInputCloud(cloud);
    // the marked records
    int cloud_size = cloud->size();
    std::vector<int> marked_indices;
    marked_indices.resize(cloud_size);

    // set to zero
    memset(marked_indices.data(), 0, sizeof(int) * cloud_size);
    std::vector<int> pointIdx;
    std::vector<float> pointquaredDistance;

    UnionFind uf(cloud_size);
    int tag_num = 1;

    for (i = 0; i < cloud_size; i++)
    {
        // Clustering process
        if (marked_indices[i] == 0) // reset to initial value if this point has not been manipulated
        {
            pointIdx.clear();
            pointquaredDistance.clear();
            cloud_kdtreeflann.radiusSearch(cloud->points[i], tolerance, pointIdx, pointquaredDistance, max_n);
            /**
            * All neighbors closest to a specified point with a query within a given radius
            * para.tolerance is the radius of the sphere that surrounds all neighbors
            * pointIdx is the resulting index of neighboring points
            * pointquaredDistance is the final square distance to adjacent points
            * pointIdx.size() is the maximum number of neighbors returned by limit
            */
            int min_tag_num = tag_num;
            for (j = 0; j < pointIdx.size(); j++)
            {
                /**
                 * find the minimum label value contained in the field points, and tag it to this cluster label.
                 */
                if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num))
                {
                    min_tag_num = marked_indices[pointIdx[j]];
                }
            }
            for (j = 0; j < pointIdx.size(); j++)
            {
                int p = pointIdx[j];
                // mark current point with the minimum tag
                if (marked_indices[p] == 0) marked_indices[p] = tag_num;
                // union: merge this point into the cluster with min_tag_num
                // find a representative point already carrying min_tag_num
                for (unsigned long k = 0; k < pointIdx.size(); k++) {
                    if (marked_indices[pointIdx[k]] == min_tag_num) {
                        uf.unite(p, pointIdx[k]);
                        break;
                    }
                }
            }
            marked_indices[i] = min_tag_num;
            tag_num++;
        }
    }

    // flatten union-find: each point gets its root as the final label
    for (i = 0; i < cloud_size; i++) {
        if (marked_indices[i] > 0) {
            marked_indices[i] = uf.find(i);
        }
    }

    std::vector<PointIndex_NumberTag> indices_tags;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    indices_tags.resize(cloud_size);

    PointIndex_NumberTag temp_index_tag;


    for (i = 0; i < cloud_size; i++)
    {
        /**
        * Put each point index and the corresponding tag value into the indices_tags
        */
        temp_index_tag.nPointIndex = i;
        temp_index_tag.nNumberTag = marked_indices[i];

        indices_tags[i] = temp_index_tag;
    }
    // sort the indices tag by NUM of tags
    sort(indices_tags.begin(), indices_tags.end(), NumberTag);

    unsigned long begin_index = 0;
    for (i = 0; i < indices_tags.size(); i++)
    {
        // Relabel each cluster
        if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag)
        {
            if ((i - begin_index) >= min_component_size)
            {
                unsigned long m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; j++)
                    inliers->indices[m++] = indices_tags[j].nPointIndex;
                cluster_indices.push_back(*inliers);
            }
            begin_index = i;
        }
    }
    // the last cluster (determine whether is a inlier)
    if ((i - begin_index) >= min_component_size)
    {
        unsigned long m = 0;
        inliers->indices.resize(i - begin_index);
        for (j = begin_index; j < i; j++)
        {
            inliers->indices[m++] = indices_tags[j].nPointIndex;
        }
        cluster_indices.push_back(*inliers);
    }
    return cluster_indices;

}
