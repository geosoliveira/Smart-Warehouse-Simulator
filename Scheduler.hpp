/* 
 * File:   Scheduler.hpp
 * Author: geoso
 *
 * Created on 10 de Março de 2021, 11:29
 */

#ifndef SCHEDULER_HPP
#define	SCHEDULER_HPP

#include <algorithm>
#include <array>
#include <numeric>
#include <queue> 
#include <random>
#include "Domain.hpp"
#include "TSP.hpp"

class Scheduler {
public:
    Scheduler() = delete;
    Scheduler(std::shared_ptr<std::vector<std::shared_ptr<Robot> > >);
    void decrease_time_of_all(std::shared_ptr<Robot>);
    bool must_compute_domain();
    std::pair<bool, std::shared_ptr<Robot> > must_compute_domain_v2();
    std::shared_ptr<Robot> next_robot_v2();
    std::shared_ptr<Robot> next_robot_v3();
    std::vector<std::shared_ptr<Robot> > next_robots();
    void prepare_v2(std::shared_ptr<Domain>);
    void prepare_v3(std::shared_ptr<Domain>);
    void prepare_v4(std::shared_ptr<Domain>);
    void update_dynamic_pos(std::vector<std::shared_ptr<Robot> >, std::shared_ptr<CostEstimator>);
    static std::tuple<int, int, float, std::vector<float> > fixed_done_cpta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, std::vector<float> > dyn_done_cpta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, std::vector<float> > dyn_done_cpta_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, std::vector<float> > nCAR_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    static std::tuple<int, int, float, std::vector<float> > greedy(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >&, CostEstimator::costing_t);
    float set_time_v2(std::shared_ptr<Robot>, std::shared_ptr<DeliverTask>, std::shared_ptr<Domain>, std::shared_ptr<CostEstimator>, CostEstimator::costing_t);
    float set_time_v2(std::shared_ptr<Robot>, std::shared_ptr<PickTask>, std::shared_ptr<Domain>, std::shared_ptr<CostEstimator>, CostEstimator::costing_t);
    float set_time_v3(std::shared_ptr<Robot>, std::shared_ptr<DeliverTask>, std::shared_ptr<Domain>, std::shared_ptr<CostEstimator>, CostEstimator::costing_t);
    float set_time_v3(std::shared_ptr<Robot>, std::shared_ptr<PickTask>, std::shared_ptr<Domain>, std::shared_ptr<CostEstimator>, CostEstimator::costing_t);
    void show_arrivals();
    void show_arrivals_v2();
    void show_arrivals_v3();
    ~Scheduler();
private:
    std::vector<std::tuple<std::shared_ptr<Robot>, float, int, int, bool> > __arrivals;
    static bool __compare_arrivals(std::tuple<std::shared_ptr<Robot>, float, int, int, bool>, std::tuple<std::shared_ptr<Robot>, float, int, int, bool>);
    static bool __compare_arrivals_v2(std::tuple<std::shared_ptr<Robot>, float, int, int, bool>, std::tuple<std::shared_ptr<Robot>, float, int, int, bool>);
    static std::tuple<float, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > > __feasible_route_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<Robot>, std::shared_ptr<CostEstimator>);
    static std::tuple<float, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<Robot> > __feasible_route_v3(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::vector<std::shared_ptr<Robot> >, std::shared_ptr<CostEstimator>);
};

#endif	/* SCHEDULER_HPP */

