/* 
 * File:   Domain.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 18:49
 */

#ifndef DOMAIN_HPP
#define	DOMAIN_HPP

#include <map>
#include "CostEstimator.hpp"
#include "Graph.hpp"
#include "PickTask.hpp"
#include "DeliverTask.hpp"
#include "Robot.hpp"
#include "TSP.hpp"

class Domain {
public:
    Domain() = delete;
    Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >, std::shared_ptr<CostEstimator>);
    Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<Robot> > >, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > >, std::shared_ptr<CostEstimator> _ce);
    void clear_domain_lists();
    void compute_depot_domain();
    void compute_domain_v1();
    void compute_domain_v4();
    void compute_dominants();
    void remove_ptask_from_dominants(std::shared_ptr<PickTask>, std::shared_ptr<Robot>);
    std::vector<std::shared_ptr<PickTask> >  get_domain_of_robot(std::shared_ptr<Robot>);
    std::vector<std::pair<std::shared_ptr<PickTask>, float> >  get_domain_of_robot_full(std::shared_ptr<Robot>);
    std::vector<std::shared_ptr<Robot> > get_dominants_of_task(std::shared_ptr<PickTask>);
    std::vector<std::pair<std::shared_ptr<Robot>, float> > get_dominants_of_task_full(std::shared_ptr<PickTask>);
    std::vector<std::shared_ptr<PickTask> >  get_domain_of_depot(std::shared_ptr<DeliverTask>);
    int get_n_primary_dominants_by_r_v2(std::shared_ptr<Robot>);
    std::map<uint16_t, std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > > get_dominants();
    std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > get_tuple_of_robot(uint16_t);
    std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > > get_tuple_of_task(uint16_t);
    std::shared_ptr<PickTask> cost_comparison(std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> >);
    std::shared_ptr<PickTask> cost_comparison_v2(std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> >);
    void show_domain_of_robot(std::shared_ptr<Robot>);
    void show_domains();
    void show_dominants();
    ~Domain();
private:
    static bool __sortByCost(std::pair<std::shared_ptr<PickTask>, float>, std::pair<std::shared_ptr<PickTask>, float>);
    std::map<uint16_t, std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > > > __domain_list;
    std::map<uint16_t, std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > > __dominant_list;
    std::map<uint16_t, std::tuple<std::shared_ptr<DeliverTask>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > > __deliver_dominant_list;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > __pick_tasks_ref;
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > __robots_ref;
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > __deliver_tasks_ref;
    std::shared_ptr<CostEstimator> __ce_ref;
};

#endif	/* DOMAIN_HPP */

