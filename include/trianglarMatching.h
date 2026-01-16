//
// Created by cyz on 2023/4/7.
//
// modified from GlobalMatch in ISPRS-J(2023) https://github.com/zexinyang/GlobalMatch

#ifndef STD_DETECTOR_TRIANGLARMATCHING_H
#define STD_DETECTOR_TRIANGLARMATCHING_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/distances.h>
#include <pcl/PolygonMesh.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <algorithm>
#include <vector>
#include <set>
#include <fstream>
#include <numeric>
#include <pcl/common/common.h>

typedef pcl::PointXY Point2D;
typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud <Point2D> Cloud2D;
typedef pcl::PointCloud <Point3D> Cloud3D;
typedef std::vector <Cloud2D, Eigen::aligned_allocator<Cloud2D> > Cloud2DVector;
typedef std::vector <Cloud3D, Eigen::aligned_allocator<Cloud3D> > Cloud3DVector;
typedef pcl::PointCloud <pcl::Normal> CloudNormal;

class TriangleMatching {
public:
    TriangleMatching() : knn_(20),
                 max_stems_for_exhaustive_search_(50),
                 edge_diff_(0.1),
                 stem_positions_src_(new Cloud3D),
                 stem_positions_tgt_(new Cloud3D) {
    }

    inline void
    setPairwiseStemPositions(const Cloud3D::ConstPtr& stem_positions_src,
                             const Cloud3D::ConstPtr& stem_positions_tgt) {
        stem_positions_src_ = stem_positions_src;
        stem_positions_tgt_ = stem_positions_tgt;
    }

    void
    estimateTransformation(Eigen::Matrix4f& transform);

    inline size_t
    getNumberOfMatches() {
        return stem_matches_.size();
    };

    // TODO: add setters and getters

private:
    struct VertexSide {
        int vertex;
        float side;
    };
    typedef std::vector<VertexSide> Triangle;

    void
    constructTriangles(const Cloud3D::ConstPtr& stem_positions,
                       std::vector<Triangle>& triangles) const;

    bool
    satisfyLocalConsistency(const Point3D& feature_src,
                            const Point3D& feature_tgt) const;

    bool
    satisfyGlobalConsistency(const std::vector<int>& pair_initial,
                             const std::vector<int>& pair_candidate);

    void
    localMatching();

    void
    globalMatching();

    /**
     * @brief Accelerate global matching by growing only a limited number of (e.g., 10k)
     * randomly sampled groups of triangle pairs. Use this function if your point cloud contains
     * a large number of trees (e.g., more than 10k trees per point cloud).
     */
    void
    randomGlobalMatching();

    Cloud3D::ConstPtr stem_positions_src_, stem_positions_tgt_;
    std::vector<Triangle> triangles_src_, triangles_tgt_;
    std::vector<std::pair<int, int>> locally_matched_pairs_;
    std::vector<int> globally_matched_pairs_;
    pcl::Correspondences stem_matches_;

    int knn_;
    int max_stems_for_exhaustive_search_;
    float edge_diff_;
};

#endif //STD_DETECTOR_TRIANGLARMATCHING_H
