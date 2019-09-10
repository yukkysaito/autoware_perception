/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "multi_object_tracker/data_association/data_association.hpp"
#include "hungarian/hungarian.h"
#include "multi_object_tracker/utils/utils.hpp"
#include "hungarian_assigner/hungarian_assigner.hpp"
#include "successive_shortest_path/successive_shortest_path.h"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>

// #define EVAL_GOOGLE_HUNGARIAN
// #define EVAL_DOTAUTO_HUNGARIAN
#define EVAL_SUCCESSIVE_SHORTEST_PATH

// #include <iostream>
DataAssociation::DataAssociation()
    : score_threshold_(0.1)
{
    can_assgin_matrix_ = Eigen::MatrixXi::Identity(20, 20);
    can_assgin_matrix_(autoware_msgs::Semantic::UNKNOWN, autoware_msgs::Semantic::UNKNOWN) = 0;
    can_assgin_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::UNKNOWN) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::TRUCK) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::BUS) = 0;
    can_assgin_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::UNKNOWN) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::CAR) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::BUS) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::UNKNOWN) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::CAR) = 0;
    can_assgin_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::TRUCK) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::UNKNOWN) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::MOTORBIKE) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::UNKNOWN) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::BICYCLE) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::UNKNOWN) = 1;
    can_assgin_matrix_(autoware_msgs::Semantic::ANIMAL, autoware_msgs::Semantic::UNKNOWN) = 1;
    max_dist_matrix_ = Eigen::MatrixXd::Constant(20, 20, 1.0);
    max_dist_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::UNKNOWN) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::CAR) = 4.5;
    max_dist_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::TRUCK) = 4.5;
    max_dist_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::BUS) = 4.5;
    max_dist_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::UNKNOWN) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::CAR) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::TRUCK) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::BUS) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::UNKNOWN) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::CAR) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::TRUCK) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::BUS) = 4.0;
    max_dist_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::UNKNOWN) = 3.0;
    max_dist_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::BICYCLE) = 3.0;
    max_dist_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::MOTORBIKE) = 3.0;
    max_dist_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::UNKNOWN) = 3.0;
    max_dist_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::BICYCLE) = 3.0;
    max_dist_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::MOTORBIKE) = 2.0;
    max_dist_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::UNKNOWN) = 2.0;
    max_dist_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::PEDESTRIAN) = 2.0;
    max_dist_matrix_(autoware_msgs::Semantic::ANIMAL, autoware_msgs::Semantic::UNKNOWN) = 1.0;
    max_area_matrix_ = Eigen::MatrixXd::Constant(20, 20, /* large number */ 10000.0);
    max_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::UNKNOWN) = 2.2 * 5.5;
    max_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::CAR) = 2.2 * 5.5;
    max_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::TRUCK) = 2.5 * 7.9;
    max_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::BUS) = 2.7 * 12.0;
    max_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::UNKNOWN) = 2.5 * 7.9;
    max_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::CAR) = 2.2 * 5.5;
    max_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::TRUCK) = 2.5 * 7.9;
    max_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::BUS) = 2.7 * 12.0;
    max_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::UNKNOWN) = 2.7 * 12.0;
    max_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::CAR) = 2.2 * 5.5;
    max_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::TRUCK) = 2.5 * 7.9;
    max_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::BUS) = 2.7 * 12.0;
    max_area_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::UNKNOWN) = 2.5;
    max_area_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::BICYCLE) = 2.5;
    max_area_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::MOTORBIKE) = 3.0;
    max_area_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::UNKNOWN) = 3.0;
    max_area_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::BICYCLE) = 2.5;
    max_area_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::MOTORBIKE) = 3.0;
    max_area_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::UNKNOWN) = 2.0;
    max_area_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::PEDESTRIAN) = 2.0;
    max_area_matrix_(autoware_msgs::Semantic::ANIMAL, autoware_msgs::Semantic::UNKNOWN) = 2.0;
    min_area_matrix_ = Eigen::MatrixXd::Constant(20, 20, /* small number */ 0.0);
    min_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::UNKNOWN) = 1.2 * 3.0;
    min_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::CAR) = 1.2 * 3.0;
    min_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::TRUCK) = 1.5 * 4.0;
    min_area_matrix_(autoware_msgs::Semantic::CAR, autoware_msgs::Semantic::BUS) = 2.0 * 5.0;
    min_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::UNKNOWN) = 1.5 * 4.0;
    min_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::CAR) = 1.2 * 3.0;
    min_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::TRUCK) = 1.5 * 4.0;
    min_area_matrix_(autoware_msgs::Semantic::TRUCK, autoware_msgs::Semantic::BUS) = 2.0 * 5.0;
    min_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::UNKNOWN) = 2.0 * 5.0;
    min_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::CAR) = 1.2 * 3.0;
    min_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::TRUCK) = 1.5 * 4.0;
    min_area_matrix_(autoware_msgs::Semantic::BUS, autoware_msgs::Semantic::BUS) = 2.0 * 5.0;
    min_area_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::UNKNOWN) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::BICYCLE) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::BICYCLE, autoware_msgs::Semantic::MOTORBIKE) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::UNKNOWN) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::BICYCLE) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::MOTORBIKE, autoware_msgs::Semantic::MOTORBIKE) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::UNKNOWN) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::PEDESTRIAN, autoware_msgs::Semantic::PEDESTRIAN) = 0.001;
    min_area_matrix_(autoware_msgs::Semantic::ANIMAL, autoware_msgs::Semantic::UNKNOWN) = 0.5;
}

bool DataAssociation::assign(const Eigen::MatrixXd &src,
                             std::unordered_map<int, int> &direct_assignment,
                             std::unordered_map<int, int> &reverse_assignment)
{
    static std::vector<double> time_ghs, time_dhs, time_ssps;
    static int dh_err_count = 0;

    std::vector<std::vector<double>> score(src.rows());
    for (int row = 0; row < src.rows(); ++row)
    {
        score.at(row).resize(src.cols());
        for (int col = 0; col < src.cols(); ++col)
        {
            score.at(row).at(col) = src(row, col);
        }
    }
    int n_rows = src.rows();
    int n_cols = src.cols();

    std::cout << "[score matrix size: " << std::setprecision(4) << std::setw(3) << n_rows << "x" << std::setw(3) << n_cols << " ";
    std::cout << std::setw(5) << 100.0 * (src.array() < 1e-5).count() / (n_rows * n_cols) << "\% are 0.]";

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Google's Hungarian
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef EVAL_GOOGLE_HUNGARIAN
    std::unordered_map<int, int> direct_assignment_gh, reverse_assignment_gh;

    std::chrono::system_clock::time_point start_time_gh, end_time_gh;
    start_time_gh = std::chrono::system_clock::now();

    // Solve
    operations_research::MaximizeLinearAssignment(score, &direct_assignment_gh, &reverse_assignment_gh);

    end_time_gh = std::chrono::system_clock::now();
    double time_gh = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end_time_gh - start_time_gh).count() / 1000.0);
    time_ghs.push_back(time_gh);

    double sum_score_gh = 0;
    for (auto it = direct_assignment_gh.cbegin(); it != direct_assignment_gh.cend(); ++it)
    {
        sum_score_gh += score[it->first][it->second];
    }

    std::cout << "\tsum score: ";
    std::cout << "(GH) " << std::setw(5) << sum_score_gh;
#endif

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DotAuto's Hungarian
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef EVAL_DOTAUTO_HUNGARIAN
    std::chrono::system_clock::time_point start_time_dh, end_time_dh;
    start_time_dh = std::chrono::system_clock::now();

    // Solve
    // autoware::fusion::hungarian_assigner::hungarian_assigner_c<256U> assigner;
    autoware::fusion::hungarian_assigner::hungarian_assigner_c<512U> assigner;
    if (n_rows <= n_cols)
    {
        assigner.set_size(n_rows, n_cols);
    }
    else
    {
        assigner.set_size(n_cols, n_rows);
    }

    for (uint64_t idx = 0U; idx < score.size(); ++idx)
    {
        const std::vector<double> &w = score[idx];
        for (uint64_t jdx = 0U; jdx < w.size(); ++jdx)
        {
            if (n_rows <= n_cols)
            {
                assigner.set_weight(-w[jdx], idx, jdx);
            }
            else
            {
                assigner.set_weight(-w[jdx], jdx, idx);
            }
        }
    }
    bool ret = assigner.assign();
    assert(ret == true);
    if (!ret)
    {
        std::string err_msg = "\nFailed to assign.\n";
        std::cout << err_msg;
        // throw err_msg;
    }

    end_time_dh = std::chrono::system_clock::now();
    double time_dh = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end_time_dh - start_time_dh).count() / 1000.0);
    time_dhs.push_back(time_dh);

    // Calculate sum of score
    double sum_score_dh = 0;
    if (n_rows <= n_cols)
    {
        for (uint i = 0U; i < n_rows; ++i)
        {
            sum_score_dh += score.at(i).at(assigner.get_assignment(i));
        }
    }
    else
    {
        for (uint i = 0U; i < n_cols; ++i)
        {
            sum_score_dh += score.at(assigner.get_assignment(i)).at(i);
        }
    }
    std::cout << " (DH) " << std::setw(5) << sum_score_dh;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Successive Shortest Path (ours)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef EVAL_SUCCESSIVE_SHORTEST_PATH
    std::unordered_map<int, int> direct_assignment_ssp, reverse_assignment_ssp;

    std::chrono::system_clock::time_point start_time_ssp, end_time_ssp;
    start_time_ssp = std::chrono::system_clock::now();

    // Solve
    assignment_problem::MaximizeLinearAssignment(score, &direct_assignment_ssp, &reverse_assignment_ssp);

    end_time_ssp = std::chrono::system_clock::now();
    double time_ssp = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end_time_ssp - start_time_ssp).count() / 1000.0);
    time_ssps.push_back(time_ssp);

    // Calculate sum of score
    double sum_score_ssp = 0;
    for (auto it = direct_assignment_ssp.cbegin(); it != direct_assignment_ssp.cend(); it++)
    {
        sum_score_ssp += score[it->first][it->second];
    }

    std::cout << " (SSP) " << std::setw(5) << sum_score_ssp;
#endif
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(EVAL_GOOGLE_HUNGARIAN) && defined(EVAL_SUSCESSIVE_SHORTEST_PATH)
    // Check result
    if (!(abs(sum_score_gh - sum_score_ssp) < 1e-5))
    {
        std::string err_msg = "\nAssertion failure. [ abs(sum_score_gh - sum_score_ssp) >= 1e-5 ]\n";
        std::cout << err_msg;
        throw err_msg;
    }
    assert(abs(sum_score_gh - sum_score_ssp) < 1e-5);
#endif

#if defined(EVAL_GOOGLE_HUNGARIAN) && defined(EVAL_DOTAUTO_HUNGARIAN)
    if (!(abs(sum_score_gh - sum_score_dh) < 1e-5))
    {
        std::string err_msg = "\nAssertion failure. [ abs(sum_score_gh - sum_score_dh) >= 1e-5 ]\n";
        std::cout << err_msg;
        // throw err_msg;
        dh_err_count += 1;
    }
    assert(abs(sum_score_gh - sum_score_dh) < 1e-5);
    std::cout << " error ratio (DH): " << std::setw(3) << dh_err_count << "/" << std::setw(3) << time_ssps.size();
#endif

    std::cout << " ---> OK";

    std::cout << "\ttime [ms]:";
#ifdef EVAL_GOOGLE_HUNGARIAN
    std::cout << " (GH) " << std::setw(5) << time_gh;
#endif
#ifdef EVAL_DOTAUTO_HUNGARIAN
    std::cout << " (DH): " << std::setw(5) << time_dh;
#endif
#ifdef EVAL_SUCCESSIVE_SHORTEST_PATH
    std::cout << " (SSP): " << std::setw(5) << time_ssp;
#endif

    // Calculate average time and std
    std::cout
        << "\t ave. time [ms]:";
#ifdef EVAL_GOOGLE_HUNGARIAN
    double time_ave_gh = std::accumulate(time_ghs.cbegin(), time_ghs.cend(), 0.0) / time_ghs.size();
    double time_var_gh = std::accumulate(time_ghs.cbegin(), time_ghs.cend(), 0.0, [time_ave_gh](double sum, const double &e) {
                             const auto temp = e - time_ave_gh;
                             return sum + temp * temp;
                         }) /
                         time_ghs.size();
    std::cout << "(GH) " << std::setw(5) << time_ave_gh << "(+ / -) " << std::setw(5) << std::sqrt(time_var_gh);
#endif
#ifdef EVAL_DOTAUTO_HUNGARIAN
    double time_ave_dh = std::accumulate(time_dhs.cbegin(), time_dhs.cend(), 0.0) / time_dhs.size();
    double time_var_dh = std::accumulate(time_dhs.cbegin(), time_dhs.cend(), 0.0, [time_ave_dh](double sum, const double &e) {
                             const auto temp = e - time_ave_dh;
                             return sum + temp * temp;
                         }) /
                         time_dhs.size();
    std::cout << "  (DH): " << std::setw(5) << time_ave_dh << " (+/-)" << std::setw(5) << std::sqrt(time_var_dh);
#endif
#ifdef EVAL_SUCCESSIVE_SHORTEST_PATH
    double time_ave_ssp = std::accumulate(time_ssps.cbegin(), time_ssps.cend(), 0.0) / time_ssps.size();
    double time_var_ssp = std::accumulate(time_ssps.cbegin(), time_ssps.cend(), 0.0, [time_ave_ssp](double sum, const double &e) {
                              const auto temp = e - time_ave_ssp;
                              return sum + temp * temp;
                          }) /
                          time_ssps.size();
    std::cout << "  (SSP): " << std::setw(5) << time_ave_ssp << " (+/-)" << std::setw(5) << std::sqrt(time_var_ssp);
#endif

    std::cout << std::endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef EVAL_GOOGLE_HUNGARIAN
    for (auto it = direct_assignment_gh.cbegin(); it != direct_assignment_gh.cend(); it++)
    {
        direct_assignment[it->first] = it->second;
    }
    for (auto it = reverse_assignment_gh.cbegin(); it != reverse_assignment_gh.cend(); it++)
    {
        reverse_assignment[it->first] = it->second;
    }
#endif
#ifdef EVAL_DOTAUTO_HUNGARIAN
    if (n_rows <= n_cols)
    {
        for (uint i = 0U; i < n_rows; ++i)
        {
            direct_assignment[i] = assigner.get_assignment(i);
            reverse_assignment[assigner.get_assignment(i)] = i;
        }
    }
    else
    {
        for (uint i = 0U; i < n_cols; ++i)
        {
            direct_assignment[assigner.get_assignment(i)] = i;
            reverse_assignment[i] = assigner.get_assignment(i);
        }
    }

#endif
#ifdef EVAL_SUCCESSIVE_SHORTEST_PATH
    for (auto it = direct_assignment_ssp.cbegin(); it != direct_assignment_ssp.cend(); it++)
    {
        direct_assignment[it->first] = it->second;
    }
    for (auto it = reverse_assignment_ssp.cbegin(); it != reverse_assignment_ssp.cend(); it++)
    {
        reverse_assignment[it->first] = it->second;
    }
#endif

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    for (auto itr = direct_assignment.begin(); itr != direct_assignment.end();)
    {
        if (src(itr->first, itr->second) < score_threshold_)
        {
            itr = direct_assignment.erase(itr);
            continue;
        }
        else
        {
            ++itr;
        }
    }
    for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end();)
    {
        if (src(itr->second, itr->first) < score_threshold_)
        {
            itr = reverse_assignment.erase(itr);
            continue;
        }
        else
        {
            ++itr;
        }
    }
}

Eigen::MatrixXd DataAssociation::calcScoreMatrix(const autoware_msgs::DynamicObjectWithFeatureArray &measurements,
                                                 const std::list<std::shared_ptr<Tracker>> &trackers)
{
    Eigen::MatrixXd score_matrix = Eigen::MatrixXd::Zero(trackers.size(), measurements.feature_objects.size());
    size_t tracker_idx = 0;
    for (auto tracker_itr = trackers.begin(); tracker_itr != trackers.end(); ++tracker_itr, ++tracker_idx)
    {
        for (size_t measurement_idx = 0; measurement_idx < measurements.feature_objects.size(); ++measurement_idx)
        {
            double score = 0.0;
            if (can_assgin_matrix_((*tracker_itr)->getType(), measurements.feature_objects.at(measurement_idx).object.semantic.type))
            {
                double max_dist = max_dist_matrix_((*tracker_itr)->getType(), measurements.feature_objects.at(measurement_idx).object.semantic.type);
                double max_area = max_area_matrix_((*tracker_itr)->getType(), measurements.feature_objects.at(measurement_idx).object.semantic.type);
                double min_area = min_area_matrix_((*tracker_itr)->getType(), measurements.feature_objects.at(measurement_idx).object.semantic.type);
                double dist = getDistance(measurements.feature_objects.at(measurement_idx).object.state.pose.pose.position, (*tracker_itr)->getPosition(measurements.header.stamp));
                double area = utils::getArea(measurements.feature_objects.at(measurement_idx).object.shape);
                score = (max_dist - std::min(dist, max_dist)) / max_dist;

                if (max_dist < dist)
                    score = 0.0;
                if (area < min_area || max_area < area)
                    score = 0.0;
                // if ((*tracker_itr)->getType() == measurements.feature_objects.at(measurement_idx).object.semantic.type &&
                //     measurements.feature_objects.at(measurement_idx).object.semantic.type != autoware_msgs::Semantic::UNKNOWN)
                //     score += 1.0;
                // if (measurements.feature_objects.at(measurement_idx).object.semantic.type != autoware_msgs::Semantic::UNKNOWN)
                //     score += 1.0;
            }
            score_matrix(tracker_idx, measurement_idx) = score;
        }
    }

    return score_matrix;
}

double DataAssociation::getDistance(const geometry_msgs::Point &measurement,
                                    const geometry_msgs::Point &tracker)
{
    const double diff_x = tracker.x - measurement.x;
    const double diff_y = tracker.y - measurement.y;
    // const double diff_z = tracker.z - measurement.z;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}
