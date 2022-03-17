/* 
 * File:   Scheduler.cpp
 * Author: geoso
 * 
 * Created on 10 de Março de 2021, 11:29
 */

#include "Scheduler.hpp"
#include "Graph.hpp"

bool Scheduler::__compare_arrivals(std::tuple<std::shared_ptr<Robot>, float, int, int, bool> _a1, std::tuple<std::shared_ptr<Robot>, float, int, int, bool> _a2) {
    std::shared_ptr<Robot> robot_1 = std::get<0>(_a1);
    std::shared_ptr<Robot> robot_2 = std::get<0>(_a2);
    float time_robot_1 = std::get<1>(_a1);
    float time_robot_2 = std::get<1>(_a2);
    int n_dominants_robot_1 = std::get<2>(_a1);
    int n_dominants_robot_2 = std::get<2>(_a2);
    if (time_robot_1 == time_robot_2) {
        if (n_dominants_robot_1 == n_dominants_robot_2)
            return (robot_1->get_id() < robot_2->get_id());
        return (n_dominants_robot_1 > n_dominants_robot_2);
    }
    return (time_robot_1 < time_robot_2);
}

bool Scheduler::__compare_arrivals_v2(std::tuple<std::shared_ptr<Robot>, float, int, int, bool> _a1, std::tuple<std::shared_ptr<Robot>, float, int, int, bool> _a2) {
    std::shared_ptr<Robot> robot_1 = std::get<0>(_a1);
    std::shared_ptr<Robot> robot_2 = std::get<0>(_a2);
    float time_robot_1 = std::get<1>(_a1);
    float time_robot_2 = std::get<1>(_a2);
    int iteration_1 = std::get<3>(_a1);
    int iteration_2 = std::get<3>(_a2);
    if ((int)(robot_1->get_state()) == (int)(robot_2->get_state())) {
        if (iteration_1 == iteration_2)
            return (time_robot_1 < time_robot_2);
        return (iteration_1 < iteration_2);
    }
    return ((int)(robot_1->get_state()) > (int)(robot_2->get_state()));
}

Scheduler::Scheduler(std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots_ref) {
    int n_robots = _robots_ref->size();    
    for (int i = 0; i < n_robots; i++) {
        this->__arrivals.push_back(std::make_tuple((*_robots_ref)[i], 0.0, 0, 0, false));
    }
#ifdef DEBUG
    std::cout << "Arrivals list (after Constructor):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
}

void Scheduler::decrease_time_of_all(std::shared_ptr<Robot> _robot) {
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    std::shared_ptr<Robot> found_robot = std::get<0>(this->__arrivals[addr]);
    float tm = std::get<1>(this->__arrivals[addr]);
    
    // Atualiza
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float cur_tm = std::get<1>(this->__arrivals[i]);
        int n_dominants = std::get<2>(this->__arrivals[i]);
        float new_tm = (cur_tm - tm < 0.0) ? 0.0 : cur_tm - tm;
        bool must_compute_domain;
        if (new_tm == 0 && (cur_tm != 0.0 || tm != 0.0) && rbt->get_id() != found_robot->get_id())
            must_compute_domain = true;
        else
            must_compute_domain = std::get<4>(this->__arrivals[i]);
        this->__arrivals[i] = std::make_tuple(rbt, new_tm, n_dominants, 0, must_compute_domain);
    }
}

std::tuple<int, int, float, std::vector<float> > Scheduler::nCAR_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(_pick_tasks_ref);
    std::vector<std::shared_ptr<Robot> > robots = *(_robots_ref);
    
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    int n_visits_to_depot = 0;
    float c_total = 0.0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > t;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > not_t = _pick_tasks_ref;
    std::vector<float> it_durations;
    
    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
        cur_task->set_current_state(PickTask::unused);
    }
    int ptr = 1, idx = 0;
    for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
        cur_task->set_current_state(PickTask::waiting);
    }
    idx++;
    
    int robot_id = 0;
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            bool can_execute = false;
            for (int i = 1; i < _pick_tasks_ref->size(); i++) {
                if ((*_pick_tasks_ref)[i]->get_current_state() == PickTask::waiting && robots[robot_id]->get_current_dataset_capacity() >= (*_pick_tasks_ref)[i]->get_demand()) {
                    can_execute = true;
                    break;
                } 
            }
            if (can_execute) {
                if (robots[robot_id]->get_n_tasks() && robots[robot_id]->get_last_task()->get_task_type() == Task::deliver) {
                    robots[robot_id]->remove_last_task();
                    --n_visits_to_depot;
                }
            }
            else {
                robots[robot_id]->reset_current_capacity();
            }
            
            auto tpl = Scheduler::__feasible_route_v2(not_t, robots[robot_id], ce);
            float p = std::get<0>(tpl);
            t = std::get<1>(tpl);
            not_t = std::get<2>(tpl);
            std::shared_ptr<Graph> g(new Graph(t, ce));
            auto circuit = g->tsp_christofides_v2(robots[robot_id]);
            std::vector<std::shared_ptr<Task> > cur_path = std::get<0>(circuit);
            c_total += std::get<1>(circuit) * RSE::map_scale;
            robots[robot_id]->add_scheduling_cost(std::get<1>(circuit) * RSE::map_scale);
            for (int j = 0; j < cur_path.size(); j++) {
                robots[robot_id]->add_task_ref(cur_path[j]);
                if (cur_path[j]->get_task_type() == Task::pick) {
                    robots[robot_id]->decrease_from_current_capacity(cur_path[j]->get_demand());
                    float lifting_time = (cur_path[j]->get_height() + 1) / robots[robot_id]->get_max_lifting_speed() * RSE::map_scale;
                    c_total += lifting_time;
                    robots[robot_id]->add_scheduling_cost(lifting_time);
                }
            }
            // Setting position
            std::shared_ptr<Task> last_task = cur_path[cur_path.size()-1];
            robots[robot_id]->set_position(last_task->get_position());
            for (int j = 0; j < (*t).size(); j++) {
                (*t)[j]->set_current_state(PickTask::assigned);
            }
#ifdef DEBUG
            std::cout << "Returned circuit: ";
            for (int i = 0; i < cur_path.size(); i++) {
                std::shared_ptr<Task> cur_task = cur_path[i];
                if (cur_task->get_task_type() == Task::pick)
                    std::cout << "t" << cur_task->get_id() << " ";
                else if (cur_task->get_task_type() == Task::deliver)
                    std::cout << "d" << cur_task->get_id() << " ";
            }
            std::cout << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(*not_t);
#endif
            robot_id = (robot_id == robots.size() - 1) ? 0 : robot_id + 1;
            n_visits_to_depot++;
            n_assigned += (float)(cur_path.size() - 1);
            std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
            if (n_assigned == total_tasks) break;
            int count = 0;
            for (int i = 1; i < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
                if (cur_task->get_current_state() == PickTask::waiting) count++;
            }
            if (count == 0) break;
        }
        int count = 0;
        for (int i = 1; i < _pick_tasks_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
            if (cur_task->get_current_state() == PickTask::waiting) count++;
        }

        if (count == 0) {
            for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                cur_task->set_current_state(PickTask::waiting);
            }
            idx++;
        }     
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        it_durations.push_back((float)(duration.count()) / 1000000.0);
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, it_durations);
}

bool Scheduler::must_compute_domain() {
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
    // arrivals_list é ordenado em prepare() e set_time(), assim, varre a lista e retorna o booleano com dominância maior que zero
    bool must_compute_domain;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        int n_dominants = std::get<2>(this->__arrivals[i]);
        if (n_dominants) {
            must_compute_domain = std::get<4>(this->__arrivals[i]);
#ifdef DEBUG
            std::cout << "Returning value: " << must_compute_domain << std::endl;
#endif
            break;
        }
    }
    return must_compute_domain;
}

std::pair<bool, std::shared_ptr<Robot> > Scheduler::must_compute_domain_v2() {
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
    // arrivals_list é ordenado em prepare() e set_time(), assim, varre a lista e retorna o booleano com dominância maior que zero
    bool must_compute_domain;
    std::shared_ptr<Robot> rbt;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        int n_dominants = std::get<2>(this->__arrivals[i]);
        if (n_dominants) {
            rbt = std::get<0>(this->__arrivals[i]);
            must_compute_domain = std::get<4>(this->__arrivals[i]);
#ifdef DEBUG
            std::cout << "Returning value: " << must_compute_domain << std::endl;
#endif
            break;
        }
    }
    return std::make_pair(must_compute_domain, rbt);
}

std::shared_ptr<Robot> Scheduler::next_robot_v2() {
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
     // arrivals_list é ordenado em prepare() e set_time(), assim, varre a lista e retorna o melhor robô com dominância maior que zero
    std::shared_ptr<Robot> best_robot = NULL;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        int n_dominants = std::get<2>(this->__arrivals[i]);
        if (n_dominants) {
            best_robot = std::get<0>(this->__arrivals[i]);
#ifdef DEBUG
            std::cout << "Returning robot: r" << best_robot->get_id() << std::endl;
#endif
            break;
        }
    }
    return best_robot;
}

std::shared_ptr<Robot> Scheduler::next_robot_v3() {
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals_v2();
    std::cout << std::endl;
#endif
    
     // arrivals_list é ordenado em prepare() e set_time(), assim, varre a lista e retorna o melhor robô com dominância maior que zero
    std::shared_ptr<Robot> best_robot;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        int n_dominants = std::get<2>(this->__arrivals[i]);
        if (n_dominants) {
            best_robot = std::get<0>(this->__arrivals[i]);
            if (best_robot->get_state() == Robot::idle) {
#ifdef DEBUG
                std::cout << "Returning robot: r" << best_robot->get_id() << std::endl;
#endif
                return best_robot;
            }
        }
    }
    return NULL;
}

void Scheduler::prepare_v2(std::shared_ptr<Domain> _dom) {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int it = std::get<3>(this->__arrivals[i]);
        this->__arrivals[i] = std::make_tuple(rbt, tm, _dom->get_n_primary_dominants_by_r_v2(rbt), it, false);
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    //[]( float acc, std::pair<uint16_t, float> p ) { return ( acc + p.second ); }
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
}

void Scheduler::prepare_v4(std::shared_ptr<Domain> _dom) {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int it = std::get<3>(this->__arrivals[i]);
        bool must_calc_domain = std::get<4>(this->__arrivals[i]);
        this->__arrivals[i] = std::make_tuple(rbt, tm, _dom->get_n_primary_dominants_by_r_v2(rbt), it, must_calc_domain);
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    //[]( float acc, std::pair<uint16_t, float> p ) { return ( acc + p.second ); }
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
}

void Scheduler::prepare_v3(std::shared_ptr<Domain> _dom) {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int it = std::get<3>(this->__arrivals[i]);
        this->__arrivals[i] = std::make_tuple(rbt, tm, _dom->get_n_primary_dominants_by_r_v2(rbt), it, false);
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list:" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
}

//dyn domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, std::vector<float> > Scheduler::dyn_done_cpta_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    bool realocate = false;
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > dominated_tasks;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::vector<float> it_durations;
    
    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
        cur_task->set_current_state(PickTask::unused);
    }
    int ptr = 1, idx = 0;
    for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
        cur_task->set_current_state(PickTask::waiting);
    }
    idx++;
    
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        auto start = std::chrono::high_resolution_clock::now();
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        dom->compute_domain_v4();
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v3(dom);
        for (int i = 0; i < _robots_ref->size(); i++) {
#ifdef DEBUG
            std::cout << "Getting next robot" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", state = " << cur_robot->get_state() <<
                    ", capacity = " << cur_robot->get_current_dataset_capacity() << ")" << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot_full(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants) {
                //cur_task = (n_dominants == 1) ? cur_task = dominated_tasks[0].first : dom->cost_comparison(cur_robot, dominated_tasks);
                cur_task = (n_dominants == 1) ?
                    ((dominated_tasks[0].first->get_current_state() == PickTask::assigned) ? NULL : dominated_tasks[0].first ) :
                        dom->cost_comparison(cur_robot, dominated_tasks);
                if (!cur_task) continue;
                if (cur_robot->get_state() == Robot::working) {
                    int n_tasks = cur_robot->get_n_tasks();
                    float cost_penultimate_to_last, cost_penultimate_to_current, cost_current_to_last;
                    std::shared_ptr<Task> penultimate_task = cur_robot->get_task_ref(n_tasks - 2);
                    cost_penultimate_to_current = (penultimate_task) ?
                        ce->get_sore_cost(penultimate_task->get_position(), cur_task, cur_robot) :
                        ce->get_sore_cost(cur_robot->get_initial_position(), cur_task, cur_robot);
                    std::shared_ptr<Task> last_task = cur_robot->get_last_task();
                    if (last_task->get_task_type() == Task::pick) {
                        std::vector<std::shared_ptr<PickTask> >::iterator it = std::find_if((*_pick_tasks_ref).begin(), (*_pick_tasks_ref).end(), 
                                [&last_task](std::shared_ptr<PickTask> a1){ return a1->get_id() == last_task->get_id();} );
                        if (it == (*_pick_tasks_ref).end()) { // encontrou
                            std::cerr << "OOps..." << std::endl;
                            exit(1);
                        }
                        std::shared_ptr<PickTask> last_pick_task = (*it);
                        cost_current_to_last = ce->get_sore_cost(cur_task->get_position(), last_pick_task, cur_robot);
                        cost_penultimate_to_last = (penultimate_task) ?
                            ce->get_sore_cost(penultimate_task->get_position(), last_pick_task, cur_robot) :
                            ce->get_sore_cost(cur_robot->get_initial_position(), last_pick_task, cur_robot);
                        if (cost_penultimate_to_current + cost_current_to_last < cost_penultimate_to_last + cost_current_to_last) {
                            cur_robot->remove_last_task();
                            cur_robot->increase_on_current_capacity(last_pick_task->get_demand());
                            (penultimate_task) ?
                                cur_robot->decrease_from_scheduling_cost(ce->get_cost_v2(penultimate_task->get_position(), last_pick_task, cur_robot)) :
                                cur_robot->decrease_from_scheduling_cost(ce->get_cost_v2(cur_robot->get_initial_position(), last_pick_task, cur_robot));
                            
                            last_pick_task->set_current_state(PickTask::waiting);
                            n_assigned -= 1.0;
                            //std::cout << "Realocou! Removeu pick task\n";
                            //realocate = true;
                        }
                        else {
                            auto tpl_robot_to_last_task = ce->nearest_robot(last_pick_task, cur_robot);
                            auto tpl_robot_to_curr_task = ce->nearest_robot(cur_task, cur_robot);
                            std::shared_ptr<Robot> another_robot_to_last = std::get<0>(tpl_robot_to_last_task);
                            std::shared_ptr<Robot> another_robot_to_curr = std::get<0>(tpl_robot_to_curr_task);
                            float cost_nearest_robot_last_task = std::get<1>(tpl_robot_to_last_task);
                            float cost_nearest_robot_curr_task = std::get<1>(tpl_robot_to_curr_task);
                            auto tpl_best_delivery = ce->nearest_deliver_v2(cur_robot, cur_robot);
                            float cost_best_delivery = std::get<1>(tpl_best_delivery);
                            std::shared_ptr<DeliverTask> best_deliver = std::get<0>(tpl_best_delivery);
                            if (cost_nearest_robot_last_task < cost_nearest_robot_curr_task
                                    && another_robot_to_last
                                    && cost_nearest_robot_last_task < cost_penultimate_to_last) {
                                cur_robot->remove_last_task();
                                cur_robot->increase_on_current_capacity(last_pick_task->get_demand());
                                //(penultimate_task) ?
                                //    cur_robot->decrease_from_scheduling_cost(ce->get_cost_v2(penultimate_task->get_position(), last_task, cur_robot)) :
                                //    cur_robot->decrease_from_scheduling_cost(ce->get_cost_v2(cur_robot->get_initial_position(), last_task, cur_robot));
                                last_pick_task->set_current_state(PickTask::waiting);
                                n_assigned -= 1.0;
                                //float cost_to_task = ce->get_cost_v2(another_robot_to_last, last_pick_task, another_robot_to_last);
                                //c_total += cost_to_task;
                                //another_robot_to_last->add_scheduling_cost(cost_to_task);
                                another_robot_to_last->add_task_ref(last_pick_task);
                                another_robot_to_last->decrease_from_current_capacity(last_pick_task->get_demand());
                                another_robot_to_last->set_state(Robot::working);
                                last_pick_task->set_current_state(PickTask::assigned);
                                //dom->remove_ptask_from_dominants(cur_task, cur_robot);
                                n_assigned += 1.0;
                                std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                                        (n_assigned / total_tasks)*100 << "% Completed...\r";
                                float arrival_time = sch->set_time_v3(another_robot_to_last, last_pick_task, dom, ce, _costing_type);
                                //std::cout << "Realocou! Atribuiu last pick task a outro robô\n";
                                //last_pick_task->show_me();
                                //realocate = true;
                            }
                            /*else if (cost_nearest_robot_curr_task < cost_nearest_robot_last_task
                                    && another_robot_to_curr
                                    && cost_nearest_robot_curr_task < cost_penultimate_to_last) {
                                another_robot_to_curr->add_task_ref(cur_task);
                                another_robot_to_curr->decrease_from_current_capacity(cur_task->get_demand());
                                another_robot_to_curr->set_state(Robot::working);
                                cur_task->set_current_state(PickTask::assigned);
                                //dom->remove_ptask_from_dominants(cur_task, cur_robot);
                                n_assigned += 1.0;
                                std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                                        (n_assigned / total_tasks)*100 << "% Completed...\r";
                                float arrival_time = sch->set_time_v3(another_robot_to_curr, cur_task, dom, ce, _costing_type);
                                std::cout << "Realocou! Atribuiu curr pick task a outro robô\n";
                                cur_task->show_me(); getchar();
                                continue;
                            }
                            else if (cost_nearest_robot_curr_task < cost_current_to_last
                                    && cost_nearest_robot_curr_task > cost_penultimate_to_last
                                    && cost_nearest_robot_curr_task < cost_penultimate_to_last) {
                                std::cout << "sim\n"; getchar();
                            }*/
                        }
                    }
                    else if (last_task->get_task_type() == Task::deliver) {
                        cost_current_to_last = ce->get_cost_v2(cur_task, last_task, cur_robot);
                        cost_penultimate_to_last = (penultimate_task) ?
                            ce->get_cost_v2(penultimate_task->get_position(), last_task, cur_robot) :
                            ce->get_cost_v2(cur_robot->get_initial_position(), last_task, cur_robot);
                        if (cost_penultimate_to_current + cost_current_to_last < cost_penultimate_to_last + cost_current_to_last) {
                            cur_robot->remove_last_task();
                            cur_robot->recount_current_capacity();
                            //cur_robot->decrease_from_scheduling_cost(cost_penultimate_to_last);
                            --n_visits_to_depot;
                            //std::cout << "Realocou! Removeu delivery task\n";
                            //realocate = true;
                        }
                    }
                }
                if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                    //float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    //c_total += cost_to_task;
                    //cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task);
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_state(Robot::working);
                    cur_task->set_current_state(PickTask::assigned);
                    //dom->remove_ptask_from_dominants(cur_task, cur_robot);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
                    float arrival_time = sch->set_time_v3(cur_robot, cur_task, dom, ce, _costing_type);
                }
                else {
                    //std::cout << "Encontrou delivery!\nTentando atribuir a tarefa a outro robô.\n";
                    //auto tpl = ce->nearest_robot(cur_task, cur_robot);
                    //std::shared_ptr<Robot> another_robot = std::get<0>(tpl);
                    bool attr = false;
                    do {
                        std::shared_ptr<Robot> rbt;
                        if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                            //std::cout << "\tOutra tarefa foi atribuida ao robô.\n";
                            rbt = cur_robot;
                            attr = true;
                        }
                        else {
                            /*std::cout << "\tUma tarefa foi atribuida a outro robô.\n";
                            auto tpl = ce->nearest_robot(cur_task, cur_robot);
                            rbt = std::get<0>(tpl);
                            if (!rbt) {*/
                                cur_task->set_current_state(PickTask::testing);
                                cur_task = dom->cost_comparison(cur_robot, dominated_tasks);
                                if (!cur_task) break;
                                continue;
                            //}
                        }
                        rbt->add_task_ref(cur_task);
                        rbt->decrease_from_current_capacity(cur_task->get_demand());
                        rbt->set_state(Robot::working);
                        cur_task->set_current_state(PickTask::assigned);
                        n_assigned += 1.0;
                        std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                                (n_assigned / total_tasks)*100 << "% Completed...\r";
                        float arrival_time = sch->set_time_v3(rbt, cur_task, dom, ce, _costing_type);
                        realocate = true;
                        break;
                    } while (true);
                    for (auto it = dominated_tasks.begin(); it != dominated_tasks.end(); ++it) {
                        if ((*it).first->get_current_state() == PickTask::testing) {
                            (*it).first->set_current_state(PickTask::waiting);
                        }
                    }
                    if (!attr) {
                        //cur_robot->increase_on_current_capacity(100);
                        cur_robot->reset_current_capacity();
                        auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                        std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                        cur_robot->add_task_ref(best_deliver);
                        cur_robot->set_state(Robot::working);
                        n_visits_to_depot++;
                        float arrival_time = sch->set_time_v3(cur_robot, best_deliver, dom, ce, _costing_type);
                        //std::cout << "\t\tO robô teve que executar o delivery.\n";
                    }
                    /*while (cur_robot->get_current_dataset_capacity() < cur_task->get_demand() && another_robot) {
                        //float cost_to_task = ce->get_cost_v2(another_robot, cur_task, another_robot);
                        //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                        //c_total += cost_to_task;
                        //another_robot->add_scheduling_cost(cost_to_task);
                        another_robot->add_task_ref(cur_task);
                        another_robot->decrease_from_current_capacity(cur_task->get_demand());
                        another_robot->set_state(Robot::working);
                        cur_task->set_current_state(PickTask::assigned);
                        //dom->remove_ptask_from_dominants(cur_task, cur_robot);
                        n_assigned += 1.0;
                        std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                                (n_assigned / total_tasks)*100 << "% Completed...\r";
                        float arrival_time = sch->set_time_v3(another_robot, cur_task, dom, ce, _costing_type);
                        std::cout << "\tA tarefa foi atribuida a outro robô.\n";
                        realocate = true;
                        cur_task = dom->cost_comparison(cur_robot, dominated_tasks);
                        if (!cur_task) break;
                        auto tpl = ce->nearest_robot(cur_task, cur_robot);
                        std::shared_ptr<Robot> another_robot = std::get<0>(tpl);
                    }
                    if (!cur_task || !another_robot) {
                        cur_robot->reset_current_capacity();
                        auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                        std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                        //float cost_to_best_deliver = best_deliver_tuple.second;
                        //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                        //c_total += cost_to_best_deliver;
                        //cur_robot->add_scheduling_cost(cost_to_best_deliver);
                        cur_robot->add_task_ref(best_deliver);
                        cur_robot->set_state(Robot::working);
                        n_visits_to_depot++;
                        float arrival_time = sch->set_time_v3(cur_robot, best_deliver, dom, ce, _costing_type);
                        std::cout << "\t\tO robô teve que executar o delivery.\n";
                    }
                    else if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        //float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                        //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                        //c_total += cost_to_task;
                        //cur_robot->add_scheduling_cost(cost_to_task);
                        cur_robot->add_task_ref(cur_task);
                        cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                        cur_robot->set_state(Robot::working);
                        cur_task->set_current_state(PickTask::assigned);
                        //dom->remove_ptask_from_dominants(cur_task, cur_robot);
                        n_assigned += 1.0;
                        std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                                (n_assigned / total_tasks)*100 << "% Completed...\r";
                        float arrival_time = sch->set_time_v3(cur_robot, cur_task, dom, ce, _costing_type);
                        std::cout << "\t\tO robô conseguiu pegar outra tarefa antes de executar um delivery.\n";
                    }*/
                }
                if (realocate) {
                    sch->update_dynamic_pos(sch->next_robots(), ce);
                    dom->clear_domain_lists();

                    int count = 0;
                    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
                        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
                        if (cur_task->get_current_state() == PickTask::waiting) count++;
                    }

                    if (count == 0) {
                        //std::cout << "\nidx = " << idx << std::endl;
                        //std::cout << "ptr = " << ptr << std::endl;
                        for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                            cur_task->set_current_state(PickTask::waiting);
                        }
                        idx++;
                    }

                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                    it_durations.push_back((float)(duration.count()) / 1000000.0);

                    //dom->reset_domain_lists();
                    realocate = false;
                    break;
                }
            }
        }
        sch->update_dynamic_pos(sch->next_robots(), ce);
        dom->clear_domain_lists();
        
        int count = 0;
        for (int i = 1; i < _pick_tasks_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
            if (cur_task->get_current_state() == PickTask::waiting) count++;
        }

        if (count == 0) {
            //std::cout << "\nidx = " << idx << std::endl;
            //std::cout << "ptr = " << ptr << std::endl;
            for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                cur_task->set_current_state(PickTask::waiting);
            }
            idx++;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        it_durations.push_back((float)(duration.count()) / 1000000.0);
        
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            std::shared_ptr<Task> last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            (*_robots_ref)[i]->set_position(last_task_ref->get_position());
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->nearest_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                //c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first);
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    
    //computando custos
    int i, j;
    for (i = 0; i < _robots_ref->size(); i++) {
        std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
        cur_robot->set_position(cur_robot->get_initial_position());
        std::vector<std::shared_ptr<Task> > cur_tasks = cur_robot->get_tasks_ref();
        if (cur_tasks.size()) {
            float cost = ce->get_cost_v2(cur_robot, cur_tasks[0], cur_robot) * RSE::map_scale;
            cost += (cur_tasks[0]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            for (j = 1; j < cur_tasks.size(); j++) {
                cost += ce->get_cost_v2(cur_tasks[j-1], cur_tasks[j], cur_robot) * RSE::map_scale;
                if (cur_tasks[j]->get_task_type() == Task::pick)
                    cost+= (cur_tasks[j]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            }
            cur_robot->set_scheduling_cost(cost);
            c_total += cost;
        }
    }
    
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, it_durations);
}

//dyn domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, std::vector<float> > Scheduler::dyn_done_cpta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > dominated_tasks;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::vector<float> it_durations;
    
    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
        cur_task->set_current_state(PickTask::unused);
    }
    int ptr = 1, idx = 0;
    for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
        cur_task->set_current_state(PickTask::waiting);
    }
    idx++;
    
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        auto start = std::chrono::high_resolution_clock::now();
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        dom->compute_domain_v4();
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v3(dom);
        for (int i = 0; i < _robots_ref->size(); i++) {
#ifdef DEBUG
            std::cout << "Getting next robot" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", state = " << cur_robot->get_state() <<
                    ", capacity = " << cur_robot->get_current_dataset_capacity() << ")" << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot_full(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (cur_robot->get_state() == Robot::idle && n_dominants != 0) {
                if (n_dominants == 1)
                    cur_task = dominated_tasks[0].first;
                else {
                    cur_task = dom->cost_comparison(cur_robot, dominated_tasks);
#ifdef DEBUG
                    std::cout << "\t\tBest costed tasks:" << std::endl;
                    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
                }
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    //float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float lifting_time = (cur_task->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
                    //cost_to_task += cost_to_task * RSE::map_scale + lifting_time;
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    //c_total += cost_to_task;
                    //cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task);
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    cur_robot->set_state(Robot::working);
                    cur_task->set_current_state(PickTask::assigned);
#ifdef DEBUG
                    //std::cout << "Showing domains (before removing)" << std::endl;
                    //dom->show_domains();
                    std::cout << "Showing dominants (before removing)" << std::endl;
                    dom->show_dominants();
#endif
                    dom->remove_ptask_from_dominants(cur_task, cur_robot);
#ifdef DEBUG
                    //std::cout << "Showing domains (after removing)" << std::endl;
                    //dom->show_domains();
                    std::cout << "Showing dominants (after removing)" << std::endl;
                    dom->show_dominants();
#endif
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d to task %d, updated_capacity = %d\n", cur_robot->get_id(), cur_task->get_id(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cost_to_task);
#endif
                    float arrival_time = sch->set_time_v3(cur_robot, cur_task, dom, ce, _costing_type);
#ifdef DEBUG
                    std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                    sch->show_arrivals_v3();
                    std::cout << std::endl;
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    //float cost_to_best_deliver = best_deliver_tuple.second  * RSE::map_scale;
                    //float cost_to_best_deliver = ce->get_manhattan_cost(cur_robot, best_deliver);
                    //c_total += cost_to_best_deliver;
                    //cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver);
                    cur_robot->set_state(Robot::working);
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
#endif
                    float arrival_time = sch->set_time_v3(cur_robot, best_deliver, dom, ce, _costing_type);
#ifdef DEBUG
                    std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                    sch->show_arrivals_v3();
                    std::cout << std::endl;
#endif
                }
            }
        }
        sch->update_dynamic_pos(sch->next_robots(), ce);
        dom->clear_domain_lists();
        
        int count = 0;
        for (int i = 1; i < _pick_tasks_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
            if (cur_task->get_current_state() == PickTask::waiting) count++;
        }

        if (count == 0) {
            for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                cur_task->set_current_state(PickTask::waiting);
            }
            idx++;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        it_durations.push_back((float)(duration.count()) / 1000000.0);
        
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            std::shared_ptr<Task> last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            (*_robots_ref)[i]->set_position(last_task_ref->get_position());
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->nearest_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                //float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                //c_total += cost_to_best_deliver;
                //(*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first);
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    
    //computando custos
    int i, j;
    for (i = 0; i < _robots_ref->size(); i++) {
        std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
        cur_robot->set_position(cur_robot->get_initial_position());
        std::vector<std::shared_ptr<Task> > cur_tasks = cur_robot->get_tasks_ref();
        if (cur_tasks.size()) {
            float cost = ce->get_cost_v2(cur_robot, cur_tasks[0], cur_robot) * RSE::map_scale;
            cost += (cur_tasks[0]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            for (j = 1; j < cur_tasks.size(); j++) {
                cost += ce->get_cost_v2(cur_tasks[j-1], cur_tasks[j], cur_robot) * RSE::map_scale;
                if (cur_tasks[j]->get_task_type() == Task::pick)
                    cost+= (cur_tasks[j]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            }
            cur_robot->set_scheduling_cost(cost);
            c_total += cost;
        }
    }
    
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, it_durations);
}

//domain zone based capacity and priority constrained task allocator (DoNe-CPTA)
std::tuple<int, int, float, std::vector<float> > Scheduler::fixed_done_cpta(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {  
    std::shared_ptr<CostEstimator> ce(new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref));
    ce->set_costing_type(_costing_type);
    std::shared_ptr<Domain> dom(new Domain(_pick_tasks_ref, _robots_ref, ce));
    //dom->init_domain_lists();
    std::shared_ptr<Scheduler> sch(new Scheduler(_robots_ref));
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > dominated_tasks;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::vector<float> it_durations;
    
    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
        cur_task->set_current_state(PickTask::unused);
    }
    int ptr = 1, idx = 0;
    for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
        cur_task->set_current_state(PickTask::waiting);
    }
    idx++;
    
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        auto start = std::chrono::high_resolution_clock::now();
#ifdef DEBUG
        std::cout << "Computing domains" << std::endl;
#endif
        dom->compute_domain_v4();
#ifdef DEBUG
        dom->show_domains();
#endif
#ifdef DEBUG
        std::cout << "Computing dominants" << std::endl;
#endif
        dom->compute_dominants();
#ifdef DEBUG
        dom->show_dominants();
#endif
        sch->prepare_v2(dom);
        while(true) {
#ifdef DEBUG
            std::cout << "Testing if I must have compute domain" << std::endl;
#endif
            if (sch->must_compute_domain()) {
#ifdef DEBUG
                std::cout << "\tYeah! Computing domain..." << std::endl;
#endif
                break;
            }
#ifdef DEBUG
            std::cout << "Getting next robot from queue" << std::endl;
#endif
            std::shared_ptr<Robot> cur_robot = sch->next_robot_v2();
            std::shared_ptr<PickTask> cur_task;
#ifdef DEBUG
            std::cout << "\tGot robot r" << cur_robot->get_id() << " (x = " << cur_robot->get_x() <<
                ", y = " << cur_robot->get_y() << ", capacity = " << cur_robot->get_current_dataset_capacity() <<
                ")" << std::endl;
#endif
            sch->decrease_time_of_all(cur_robot);
#ifdef DEBUG
            std::cout << "Arrivals list (after decreasing time of robot r" << cur_robot->get_id() << "):" << std::endl;
            sch->show_arrivals();
            std::cout << std::endl;
#endif
            dominated_tasks = dom->get_domain_of_robot_full(cur_robot);
#ifdef DEBUG
            std::cout << "\t\tDominated tasks:" << std::endl;
            RSE::show<std::vector<std::shared_ptr<PickTask> > >(dominated_tasks);
#endif
            uint32_t n_dominants = dominated_tasks.size();
            if (n_dominants == 0) continue;
            if (n_dominants == 1)
                cur_task = dominated_tasks[0].first;
            else {
                cur_task = dom->cost_comparison_v2(cur_robot, dominated_tasks);
#ifdef DEBUG
                std::cout << "\t\tBest costed tasks:" << std::endl;
                RSE::show<std::vector<std::shared_ptr<PickTask> > >(*best_costed_tasks_ref);
#endif
            }
            //if (cur_task) {
#ifdef DEBUG
                fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
                if (cur_robot->get_current_dataset_capacity() - cur_task->get_demand() >= 0) {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                    //float cost_to_task = ce->get_cost_v2(cur_robot, cur_task, cur_robot);
                    //float lifting_time = (cur_task->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
                    //cost_to_task += cost_to_task * RSE::map_scale + lifting_time;
                    //float cost_to_task = ce->get_manhattan_cost(cur_robot, cur_task);
                    //c_total += cost_to_task;
                    //cur_robot->add_scheduling_cost(cost_to_task);
                    cur_robot->add_task_ref(cur_task);
                    cur_robot->decrease_from_current_capacity(cur_task->get_demand());
                    float arrival_time = sch->set_time_v2(cur_robot, cur_task, dom, ce, _costing_type);
                    cur_robot->set_x(cur_task->get_x());
                    cur_robot->set_y(cur_task->get_y());
                    cur_task->set_current_state(PickTask::assigned);
#ifdef DEBUG
                    //std::cout << "Showing domains (before removing)" << std::endl;
                    //dom->show_domains();
                    std::cout << "Showing dominants (before removing)" << std::endl;
                    dom->show_dominants();
#endif
                    dom->remove_ptask_from_dominants(cur_task, cur_robot);
#ifdef DEBUG
                    //std::cout << "Showing domains (after removing)" << std::endl;
                    //dom->show_domains();
                    std::cout << "Showing dominants (after removing)" << std::endl;
                    dom->show_dominants();
#endif
                    sch->prepare_v4(dom);
                    n_assigned += 1.0;
                    std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                            (n_assigned / total_tasks)*100 << "% Completed...\r";
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: updated_position = (%d, %d), updated_capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
                    fprintf(stdout, "\t\tCost: %.f\n\n", cur_robot->get_scheduling_cost());
                    std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                    sch->show_arrivals();
                    std::cout << std::endl;
#endif
                }
                else {
#ifdef DEBUG
                    fprintf(stdout, "\tAssigning task %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
                    fprintf(stdout, "\t\tRobot %d must go to deliver first\n", cur_robot->get_id());
#endif
                    cur_robot->reset_current_capacity();
#ifdef DEBUG
                    fprintf(stdout, "\t\tRobot %d: capacity after restart = %d\n", cur_robot->get_id(), cur_robot->get_current_dataset_capacity());
#endif
                    auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    //float cost_to_best_deliver = best_deliver_tuple.second  * RSE::map_scale;
                    //c_total += cost_to_best_deliver;
                    //cur_robot->add_scheduling_cost(cost_to_best_deliver);
                    cur_robot->add_task_ref(best_deliver);
                    float arrival_time = sch->set_time_v2(cur_robot, best_deliver, dom, ce, _costing_type);
                    cur_robot->set_x(best_deliver->get_x());
                    cur_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
#ifdef DEBUG
                    fprintf(stdout, "\t\tAssigning delivery %d. Cost = %.2f\n", best_deliver->get_id(), cost_to_best_deliver);
                    std::cout << "Arrivals list (after setting new time " << arrival_time << " of robot r" << cur_robot->get_id() << "):" << std::endl;
                    sch->show_arrivals();
                    std::cout << std::endl;
#endif
                }
            //}
        }
        dom->clear_domain_lists();
        
        int count = 0;
        for (int i = 1; i < _pick_tasks_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
            if (cur_task->get_current_state() == PickTask::waiting) count++;
        }

        if (count == 0) {
            for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                cur_task->set_current_state(PickTask::waiting);
            }
            idx++;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        it_durations.push_back((float)(duration.count()) / 1000000.0);
        
        //dom->reset_domain_lists();
    }
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            std::shared_ptr<Task> last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->nearest_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                //float cost_to_best_deliver = best_deliver_tuple.second;
                //float cost_to_best_deliver = ce->get_manhattan_cost(_robots[i], best_deliver_tuple.first);
                //c_total += cost_to_best_deliver;
                //(*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first);
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    
    //computando custos
    int i, j;
    for (i = 0; i < _robots_ref->size(); i++) {
        std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
        cur_robot->set_position(cur_robot->get_initial_position());
        std::vector<std::shared_ptr<Task> > cur_tasks = cur_robot->get_tasks_ref();
        if (cur_tasks.size()) {
            float cost = ce->get_cost_v2(cur_robot, cur_tasks[0], cur_robot) * RSE::map_scale;
            cost += (cur_tasks[0]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            for (j = 1; j < cur_tasks.size(); j++) {
                cost += ce->get_cost_v2(cur_tasks[j-1], cur_tasks[j], cur_robot) * RSE::map_scale;
                if (cur_tasks[j]->get_task_type() == Task::pick)
                    cost+= (cur_tasks[j]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            }
            cur_robot->set_scheduling_cost(cost);
            c_total += cost;
        }
    }
    
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, it_durations);
}

/*std::tuple<int, int, float, std::vector<float> > Scheduler::greedy(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    CostEstimator *ce = new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref);
    ce->set_costing_type(_costing_type);
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::vector<float> it_durations;
    
    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
        cur_task->set_current_state(PickTask::unused);
    }
    int ptr = 1, idx = 0;
    for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
        cur_task->set_current_state(PickTask::waiting);
    }
    idx++;
    
    int robot_id = 0;
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            std::shared_ptr<Robot> cur_robot = (*_robots_ref)[robot_id];
#ifdef DEBUG
            fprintf(stdout, "Robot %d: position = (%d, %d), capacity = %d\n", cur_robot->get_id(), cur_robot->get_x(), cur_robot->get_y(), cur_robot->get_current_dataset_capacity());
#endif
            float min_cost = FLT_MAX;
            std::vector<std::shared_ptr<PickTask> > tasks_min_cost;
            for (int j = 0; j < _pick_tasks_ref->size(); j++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[j];
#ifdef DEBUG
                fprintf(stdout, "\tTask %d: position = (%d, %d), demand = %d\n", cur_task->get_id(), cur_task->get_x(), cur_task->get_y(), cur_task->get_demand());
#endif
                if (cur_task->get_current_state() == PickTask::waiting) {
                    float cur_cost;
                    int32_t capacity_after_pickup_i;
                    if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                        cur_cost = FLT_MAX;
                    }
                    else {
                        if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                            // Pode pegar
                            cur_cost = ce->get_cost_v2(cur_task, cur_robot, cur_robot);
                            capacity_after_pickup_i = cur_robot->get_current_dataset_capacity() - cur_task->get_demand();
                        }
                        else {
                            // Não pode pegar -- o custo é o custo de ir à estação de entrega mais próxima e, de lá, voltar à cur_task
                            auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                            cur_cost = best_deliver_tuple.second;
                            cur_cost += ce->get_cost_v2(best_deliver_tuple.first, cur_task, cur_robot);
                            capacity_after_pickup_i = cur_robot->get_total_dataset_capacity() - cur_task->get_demand();
                        }
                        // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
                        // Verificando se o robô ficará impedido de pegar outra tarefa:
                        bool can_exec_another_task = false;
                        for (int k = 0; k < _pick_tasks_ref->size(); k++) {
                            std::shared_ptr<PickTask> cur_task_of_all = (*_pick_tasks_ref)[k];
                            if (cur_task->get_id() != cur_task_of_all->get_id() &&
                                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                                can_exec_another_task = true;
                                break;
                            }
                        }
                        // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
                        if (!can_exec_another_task) {
                            auto best_deliver_tuple = ce->nearest_deliver_v2(cur_task, cur_robot);
                            cur_cost += best_deliver_tuple.second;
                        }
                    }
#ifdef DEBUG
                    std::cout << "\t\tCost = " << cur_cost << std::endl;
#endif
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        tasks_min_cost.clear();
                        tasks_min_cost.push_back(cur_task);
                    }
                    else if (cur_cost == min_cost) {
                        tasks_min_cost.push_back(cur_task);
                    }
                }
            }
            std::shared_ptr<PickTask> best_task = NULL;
            // Se não der empate, pega a primeira (e única) tarefa:
#ifdef DEBUG
            std::cout << "tasks_min_cost.size() = " << tasks_min_cost.size() << std::endl;
#endif
            if (tasks_min_cost.size() == 0) break;
            best_task = tasks_min_cost[0];
            /*if (tasks_min_cost.size() == 1) {
                best_task = tasks_min_cost[0];
            }
            // Se der empate, pega a tarefa de maior demanda que o robô consegue carregar
            else {
                int32_t larger_demand = 0;
                for (int i = 0; i < tasks_min_cost.size(); i++) {
                    std::shared_ptr<PickTask> cur_task = tasks_min_cost[i];
                    if (cur_task->get_demand() > larger_demand &&
                            cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        larger_demand = cur_task->get_demand();
                        best_task = cur_task;
                    }
                }
                // Se best_task é vazio, quer dizer que o robô não tem capacidade para carregar nenhuma tarefa.
                // Sendo assim, pega a tarefa de maior demanda
                if (!best_task) {
                    int32_t larger_demand = 0;
                    for (int i = 0; i < tasks_min_cost.size(); i++) {
                        std::shared_ptr<PickTask> cur_task = tasks_min_cost[i];
                        if (cur_task->get_demand() > larger_demand) {
                            larger_demand = cur_task->get_demand();
                            best_task = cur_task;
                        }
                    }
                }
            }
            // Se mesmo assim best_task ficar vazio, isso significa que todas as tarefas foram atribuídas
            // antes que o laço sobre o vetor de robôs terminasse. Se é assim, é só quebrar o laço
            if (!best_task) break;
            // Atribuindo a tarefa ao robô
            if (cur_robot->get_current_dataset_capacity() >= best_task->get_demand()) {
                float cost_from_robot_to_task = ce->get_cost_v2(best_task, cur_robot, cur_robot);
                float lifting_time = (best_task->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
                cost_from_robot_to_task += cost_from_robot_to_task * RSE::map_scale + lifting_time;
                c_total += cost_from_robot_to_task;
                cur_robot->add_scheduling_cost(cost_from_robot_to_task);
                cur_robot->add_task_ref(best_task);
                cur_robot->decrease_from_current_capacity(best_task->get_demand());
                cur_robot->set_x(best_task->get_x());
                cur_robot->set_y(best_task->get_y());
                best_task->set_current_state(PickTask::assigned);
                n_assigned += 1.0;
                std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
            }
            else if (cur_robot->get_total_dataset_capacity() >= best_task->get_demand()) {
                auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                float cost_to_best_deliver = best_deliver_tuple.second * RSE::map_scale;
                c_total += cost_to_best_deliver;
                cur_robot->add_scheduling_cost(cost_to_best_deliver);
                cur_robot->add_task_ref(best_deliver);
                cur_robot->reset_current_capacity();
                cur_robot->set_x(best_deliver->get_x());
                cur_robot->set_y(best_deliver->get_y());
                n_visits_to_depot++;
            }
            robot_id = (robot_id == _robots_ref->size() - 1) ? 0 : robot_id + 1;
        }
        int count = 0;
        for (int i = 1; i < _pick_tasks_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
            if (cur_task->get_current_state() == PickTask::waiting) count++;
        }

        if (count == 0) {
            for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                cur_task->set_current_state(PickTask::waiting);
            }
            idx++;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        it_durations.push_back((float)(duration.count()) / 1000000.0);
    }
    // Inserindo o melhor deliver_task no final e somando custos
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            std::shared_ptr<Task> last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->nearest_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                float cost_to_best_deliver = best_deliver_tuple.second;
                c_total += cost_to_best_deliver;
                (*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first);
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, it_durations);
}*/

std::tuple<int, int, float, std::vector<float> > Scheduler::greedy(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > _recharge_tasks_ref,
        std::shared_ptr<std::vector<std::shared_ptr<Robot> > >& _robots_ref,
        CostEstimator::costing_t _costing_type) {
    CostEstimator *ce = new CostEstimator(_pick_tasks_ref, _deliver_tasks_ref, _recharge_tasks_ref, _robots_ref);
    ce->set_costing_type(_costing_type);
    float c_total = 0.0;
    int n_visits_to_depot = 0;
    float n_assigned = 0.0, total_tasks = (float)(_pick_tasks_ref->size() - 1);
    std::vector<float> it_durations;
    
    for (int i = 1; i < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
        cur_task->set_current_state(PickTask::unused);
    }
    int ptr = 1, idx = 0;
    for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
        cur_task->set_current_state(PickTask::waiting);
    }
    idx++;
    
    while (n_assigned < total_tasks) {
    //while(!PickTask::all_assigned(_pick_tasks)) {
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 1; i < _pick_tasks_ref->size(); i++) {
            std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
            if (cur_task->get_current_state() == PickTask::waiting) {
                float min_cost = FLT_MAX;
                std::vector<std::shared_ptr<Robot> > robots_min_cost;
                for (int j = 0; j < _robots_ref->size(); j++) {
                    std::shared_ptr<Robot> cur_robot = (*_robots_ref)[j];
                    float cur_cost;
                    //int32_t capacity_after_pickup_i;
                    /*if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                        cur_cost = FLT_MAX;
                    }*/
                    //else {
                        //if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                            // Pode pegar
                            cur_cost = ce->get_cost_v2(cur_task, cur_robot, cur_robot);
                            //capacity_after_pickup_i = cur_robot->get_current_dataset_capacity() - cur_task->get_demand();
                        //}
                        /*else {
                            // Não pode pegar -- o custo é o custo de ir à estação de entrega mais próxima e, de lá, voltar à cur_task
                            auto best_deliver_tuple = ce->nearest_deliver_v2(cur_robot, cur_robot);
                            cur_cost = best_deliver_tuple.second;
                            cur_cost += ce->get_cost_v2(best_deliver_tuple.first, cur_task, cur_robot);
                            capacity_after_pickup_i = cur_robot->get_total_dataset_capacity() - cur_task->get_demand();
                        }
                        // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
                        // Verificando se o robô ficará impedido de pegar outra tarefa:
                        bool can_exec_another_task = false;
                        for (int k = 0; k < _pick_tasks_ref->size(); k++) {
                            std::shared_ptr<PickTask> cur_task_of_all = (*_pick_tasks_ref)[k];
                            if (cur_task->get_id() != cur_task_of_all->get_id() &&
                                    cur_task_of_all->get_current_state() == PickTask::waiting &&
                                    capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                                can_exec_another_task = true;
                                break;
                            }
                        }
                        // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
                        if (!can_exec_another_task) {
                            auto best_deliver_tuple = ce->nearest_deliver_v2(cur_task, cur_robot);
                            cur_cost += best_deliver_tuple.second;
                        }*/
                    //}
                    if (cur_cost < min_cost) {
                        min_cost = cur_cost;
                        robots_min_cost.clear();
                        robots_min_cost.push_back(cur_robot);
                    }
                    else if (cur_cost == min_cost) {
                        robots_min_cost.push_back(cur_robot);
                    }
                }

                std::shared_ptr<Robot> best_robot = NULL;
                
                best_robot = robots_min_cost[0];
                
                if (best_robot->get_current_dataset_capacity() < cur_task->get_demand()) {
                    auto best_deliver_tuple = ce->nearest_deliver_v2(best_robot, best_robot);
                    std::shared_ptr<DeliverTask> best_deliver = best_deliver_tuple.first;
                    //float cost_to_best_deliver = best_deliver_tuple.second * RSE::map_scale;
                    //c_total += cost_to_best_deliver;
                    //best_robot->add_scheduling_cost(cost_to_best_deliver);
                    best_robot->add_task_ref(best_deliver);
                    best_robot->reset_current_capacity();
                    best_robot->set_x(best_deliver->get_x());
                    best_robot->set_y(best_deliver->get_y());
                    n_visits_to_depot++;
                }
                
                //float cost_from_robot_to_task = ce->get_cost_v2(cur_task, best_robot, best_robot);
                //float lifting_time = (cur_task->get_height() + 1) / best_robot->get_max_lifting_speed() * RSE::map_scale;
                //cost_from_robot_to_task += cost_from_robot_to_task * RSE::map_scale + lifting_time;
                //c_total += cost_from_robot_to_task;
                //best_robot->add_scheduling_cost(cost_from_robot_to_task);
                best_robot->add_task_ref(cur_task);
                best_robot->decrease_from_current_capacity(cur_task->get_demand());
                best_robot->set_x(cur_task->get_x());
                best_robot->set_y(cur_task->get_y());
                cur_task->set_current_state(PickTask::assigned);
                n_assigned += 1.0;
                std::cerr << "[" << RSE::current_execution << " of " << RSE::number_executions << "]: " << 
                    (n_assigned / total_tasks)*100 << "% Completed...\r";
                break;

            }
            int count = 0;
            for (int i = 1; i < _pick_tasks_ref->size(); i++) {
                std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[i];
                if (cur_task->get_current_state() == PickTask::waiting) count++;
            }

            if (count == 0) {
                for (int i = 0; i < RSE::variable_tasks_at_time[idx] && ptr < _pick_tasks_ref->size(); i++) {
                    std::shared_ptr<PickTask> cur_task = (*_pick_tasks_ref)[ptr++];
                    cur_task->set_current_state(PickTask::waiting);
                }
                idx++;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        it_durations.push_back((float)(duration.count()) / 1000000.0);
    }

    // Inserindo o melhor deliver_task no final e somando custos
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks()) {
            std::shared_ptr<Task> last_task_ref = (*_robots_ref)[i]->get_task_ref((*_robots_ref)[i]->get_n_tasks() - 1);
            if (last_task_ref->get_task_type() != Task::deliver) {
                auto best_deliver_tuple = ce->nearest_deliver_v2((*_robots_ref)[i], (*_robots_ref)[i]);
                //float cost_to_best_deliver = best_deliver_tuple.second;
                //c_total += cost_to_best_deliver;
                //(*_robots_ref)[i]->add_scheduling_cost(cost_to_best_deliver);
                (*_robots_ref)[i]->add_task_ref(best_deliver_tuple.first);
                n_visits_to_depot++;
            }
        }
    }
    int n_sch_robots = 0;
    for (int i = 0; i < _robots_ref->size(); i++) {
        if ((*_robots_ref)[i]->get_n_tasks() > 0)
            n_sch_robots++;
    }
    
    //computando custos
    int i, j;
    for (i = 0; i < _robots_ref->size(); i++) {
        std::shared_ptr<Robot> cur_robot = (*_robots_ref)[i];
        cur_robot->set_position(cur_robot->get_initial_position());
        std::vector<std::shared_ptr<Task> > cur_tasks = cur_robot->get_tasks_ref();
        if (cur_tasks.size()) {
            float cost = ce->get_cost_v2(cur_robot, cur_tasks[0], cur_robot) * RSE::map_scale;
            cost += (cur_tasks[0]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            for (j = 1; j < cur_tasks.size(); j++) {
                cost += ce->get_cost_v2(cur_tasks[j-1], cur_tasks[j], cur_robot) * RSE::map_scale;
                if (cur_tasks[j]->get_task_type() == Task::pick)
                    cost+= (cur_tasks[j]->get_height() + 1) / cur_robot->get_max_lifting_speed() * RSE::map_scale;
            }
            cur_robot->set_scheduling_cost(cost);
            c_total += cost;
        }
    }
    
    return std::make_tuple(n_sch_robots, n_visits_to_depot, c_total, it_durations);
}

float Scheduler::set_time_v2(std::shared_ptr<Robot> _robot, std::shared_ptr<DeliverTask> _task, std::shared_ptr<Domain> _dom, std::shared_ptr<CostEstimator> _ce, CostEstimator::costing_t _costing_type) {
    float arrival_time;
    /*if (_costing_type == CostEstimator::euc_2d || _costing_type == CostEstimator::euc_energy ||
            _costing_type == CostEstimator::euc_time || _costing_type == CostEstimator::euc_time_energy)
        arrival_time = _ce->get_euclidean_time_cost_v2(_robot, _task, _robot);
    else 
        arrival_time = _ce->get_manhattan_time_cost_v2(_robot, _task, _robot);*/
    arrival_time = _ce->get_cost_v2(_robot, _task, _robot);
                
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    // Atualiza e diz que precisa calcular o domínio
    std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[addr]);
    int n_dominants = std::get<2>(this->__arrivals[addr]);
    this->__arrivals[addr] = std::make_tuple(rbt, arrival_time, n_dominants, 0, true);
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    return arrival_time;
}

float Scheduler::set_time_v2(std::shared_ptr<Robot> _robot, std::shared_ptr<PickTask> _task, std::shared_ptr<Domain> _dom, std::shared_ptr<CostEstimator> _ce, CostEstimator::costing_t _costing_type) {
    float arrival_time;
    /*if (_costing_type == CostEstimator::euc_2d || _costing_type == CostEstimator::euc_energy ||
            _costing_type == CostEstimator::euc_time || _costing_type == CostEstimator::euc_time_energy)
        arrival_time = _ce->get_euclidean_time_cost_v2(_robot, _task, _robot);
    else 
        arrival_time = _ce->get_manhattan_time_cost_v2(_robot, _task, _robot);*/
    arrival_time = _ce->get_cost_v2(_robot, _task, _robot);
                
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    // Atualiza e diz que precisa calcular o domínio
    std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[addr]);
    int n_dominants = std::get<2>(this->__arrivals[addr]);
    this->__arrivals[addr] = std::make_tuple(rbt, arrival_time, n_dominants, 0, true);
    
    // Se a tarefa _task estiver no domínio de outro robô s, então diz que, ao chegar em s, precisa calcular o domínio
    std::vector<std::shared_ptr<Robot> > dominants = _dom->get_dominants_of_task(_task);
    for (int i = 0; i < dominants.size(); i++) {
        std::shared_ptr<Robot> robot_in_dominants = dominants[i];
        if (robot_in_dominants->get_id() != _robot->get_id()) {
            //Procura o endereço de robot_in_dominants em arrivals
            for (int j = 0; j < this->__arrivals.size(); j++) {
                std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[j]);
                if (rbt->get_id() == robot_in_dominants->get_id()) { //Achou
                    float tm = std::get<1>(this->__arrivals[j]);
                    int n_dominants = std::get<2>(this->__arrivals[j]);
                    this->__arrivals[j] = std::make_tuple(robot_in_dominants, tm, n_dominants, 0, true);
                    break;
                }
            }
        }
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    return arrival_time;
}

float Scheduler::set_time_v3(std::shared_ptr<Robot> _robot, std::shared_ptr<DeliverTask> _task, std::shared_ptr<Domain> _dom, std::shared_ptr<CostEstimator> _ce, CostEstimator::costing_t _costing_type) {
    float arrival_time;
    /*if (_costing_type == CostEstimator::euc_2d || _costing_type == CostEstimator::euc_energy ||
            _costing_type == CostEstimator::euc_time || _costing_type == CostEstimator::euc_time_energy)
        arrival_time = _ce->get_euclidean_time_cost_v2(_robot, _task, _robot);
    else 
        arrival_time = _ce->get_manhattan_time_cost_v2(_robot, _task, _robot);*/
    arrival_time = _ce->get_cost_v2(_robot, _task, _robot);
                
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    // Atualiza
    std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[addr]);
    int iteration = std::get<3>(this->__arrivals[addr]);
    this->__arrivals[addr] = std::make_tuple(rbt, arrival_time, 0, iteration, false);
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
    return arrival_time;
}

float Scheduler::set_time_v3(std::shared_ptr<Robot> _robot, std::shared_ptr<PickTask> _task, std::shared_ptr<Domain> _dom, std::shared_ptr<CostEstimator> _ce, CostEstimator::costing_t _costing_type) {
    float arrival_time;
    /*if (_costing_type == CostEstimator::euc_2d || _costing_type == CostEstimator::euc_energy ||
            _costing_type == CostEstimator::euc_time || _costing_type == CostEstimator::euc_time_energy)
        arrival_time = _ce->get_euclidean_time_cost_v2(_robot, _task, _robot);
    else 
        arrival_time = _ce->get_manhattan_time_cost_v2(_robot, _task, _robot);*/
    arrival_time = _ce->get_cost_v2(_robot, _task, _robot);
                
    //Primeiro, procura o endereço do robô em arrivals
    int addr;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_id() == _robot->get_id()) {
            addr = i;
            break;
        }
    }
    
    // Atualiza e diz que precisa calcular o domínio
    std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[addr]);
    int iteration = std::get<3>(this->__arrivals[addr]);
    this->__arrivals[addr] = std::make_tuple(rbt, arrival_time, 0, iteration, false);
    
#ifdef DEBUG
    std::cout << "Arrivals list (before sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    //Ordena arrivals_deref
    std::sort(this->__arrivals.begin(), this->__arrivals.end(), Scheduler::__compare_arrivals);
    
#ifdef DEBUG
    std::cout << "Arrivals list (after sorting):" << std::endl;
    this->show_arrivals();
    std::cout << std::endl;
#endif
    
    return arrival_time;
}

Scheduler::~Scheduler() {
    this->__arrivals.clear();
}

void Scheduler::show_arrivals() {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int n_dominants = std::get<2>(this->__arrivals[i]);
        bool lock = std::get<4>(this->__arrivals[i]);
        std::cout << "\t[Robot: r" << rbt->get_id() << ", Time: " << tm << ", Dominants: " << n_dominants << ", Must calc. domain: " << lock << "]" << std::endl;
    }
}

void Scheduler::show_arrivals_v2() {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int n_dominants = std::get<2>(this->__arrivals[i]);
        bool lock = std::get<4>(this->__arrivals[i]);
        std::cout << "\t[Robot: r" << rbt->get_id() << ", (x = " << rbt->get_x() << ", y = " << rbt->get_y() << "), State: " << rbt->get_state() << ", Time: " << tm << ", Dominants: " << n_dominants << std::endl;
    }
}

void Scheduler::show_arrivals_v3() {
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]); 
        int iteration = std::get<3>(this->__arrivals[i]);
        bool lock = std::get<4>(this->__arrivals[i]);
        std::cout << "\t[Robot: r" << rbt->get_id() << ", (x = " << rbt->get_x() << ", y = " << rbt->get_y() << "), State: " << rbt->get_state() << ", Time: " << tm << ", Iteration: " << iteration << std::endl;
    }
}

std::tuple<float, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > >
Scheduler::__feasible_route_v2(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref, std::shared_ptr<Robot> _robot, std::shared_ptr<CostEstimator> _ce) {
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(_pick_tasks_ref);
    
    std::shared_ptr<Graph> g_main(new Graph(_pick_tasks_ref, _ce));
    auto snc_tpl_main = g_main->single_node_cycle_v2(_robot);
    //float p_min = std::get<1>(snc_tpl_main);
    float p_min = FLT_MAX;
    //float best_cost = FLT_MAX;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > t_min, not_t_min;
    bool changed = false;
    for (int i = 0; i < pick_tasks.size(); i++) {
        for (int j = 0; j < pick_tasks.size(); j++) {
            if (pick_tasks[j]->get_current_state() == PickTask::testing)
                pick_tasks[j]->set_current_state(PickTask::waiting);

        }
        std::shared_ptr<PickTask> cur_task = pick_tasks[i];
        if (cur_task->get_current_state() != PickTask::waiting) continue;
        std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > nT(new std::vector<std::shared_ptr<PickTask> >());
        std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > not_nT = PickTask::copy_picktasks_v2(*_pick_tasks_ref);
        nT->push_back(cur_task);
        std::shared_ptr<Graph> g_nT(new Graph(nT, _ce));
        PickTask::delete_picktasks_v2(not_nT, cur_task);
        std::shared_ptr<Graph> g_not_nT(new Graph(not_nT, _ce));
        int32_t dT = cur_task->get_demand();
        cur_task->set_current_state(PickTask::testing);
        //std::cout << i <<"_robot->get_current_dataset_capacity() = " << _robot->get_current_dataset_capacity() << std::endl;
        while(dT < _robot->get_current_dataset_capacity()) {
            //std::cout << "\tdT = " << dT << std::endl;
            std::shared_ptr<PickTask> best_feasible_task = g_not_nT->restricted_nearest_neighbor_v2(cur_task, dT, _robot);
            if (!best_feasible_task)
                break;
            best_feasible_task->set_current_state(PickTask::testing);
            dT += best_feasible_task->get_demand();
            nT->push_back(best_feasible_task);
            PickTask::delete_picktasks_v2(not_nT, best_feasible_task);
            if (nT->size() == RSE::route_size) break;
        }
        g_not_nT.reset();
        g_nT.reset();
        g_nT = std::make_shared<Graph>(nT, _ce);
        g_not_nT = std::make_shared<Graph>(not_nT, _ce);
        //std::cout << "euler tour" << std::endl;
        auto ec_tpl = g_nT->euler_tour_v3(_robot);
        //std::cout << "single node cycle" << std::endl;
        auto snc_tpl = g_not_nT->single_node_cycle_v2(_robot);
        //std::cout << "ok" << std::endl;
        float p = std::get<1>(ec_tpl) + std::get<1>(snc_tpl);
        if (p <= p_min) {
        //if (std::get<1>(snc_tpl) < p_min && std::get<1>(ec_tpl) < best_cost) {
            //best_cost = std::get<1>(ec_tpl);
            p_min = p;
            t_min = nT;
            not_t_min = not_nT;
            changed = true;
        }
        g_not_nT.reset();
        g_nT.reset();
    }
    if (!changed) {
        fprintf(stderr, "Cannot find a route...\n");
        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*_pick_tasks_ref);
        exit(1);
        /*t_min.reset();
        not_t_min.reset();
        return std::make_tuple(FLT_MAX, t_min, not_t_min);*/
    }
    g_main.reset();
#ifdef DEBUG
    std::cout << "Returned t_min: ";
    for (int i = 0; i < t_min->size(); i++) {
        std::shared_ptr<Task> cur_task = (*t_min)[i];
        std::cout << "t" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
    std::cout << "Returned not_t_min: ";
    for (int i = 0; i < not_t_min->size(); i++) {
        std::shared_ptr<Task> cur_task = (*not_t_min)[i];
        std::cout << "t" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
#endif
    return std::make_tuple(p_min, t_min, not_t_min);
}


std::tuple<float, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<std::vector<std::shared_ptr<PickTask> > >, std::shared_ptr<Robot> >
Scheduler::__feasible_route_v3(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref, std::vector<std::shared_ptr<Robot> > _robots, std::shared_ptr<CostEstimator> _ce) {
    std::vector<std::shared_ptr<PickTask> > pick_tasks = *(_pick_tasks_ref);
    
    float p_min = FLT_MAX;
    //float best_cost = FLT_MAX;
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > t_min, not_t_min;
    std::shared_ptr<Robot> best_robot;
    bool changed = false;
    for (int i = 0; i < _robots.size(); i++) {
        std::shared_ptr<Robot> cur_robot = _robots[i];
        for (int j = 0; j < pick_tasks.size(); j++) {
            for (int k = 0; k < pick_tasks.size(); k++) {
                if (pick_tasks[k]->get_current_state() == PickTask::testing)
                    pick_tasks[k]->set_current_state(PickTask::waiting);

            }
            std::shared_ptr<PickTask> cur_task = pick_tasks[j];
            if (cur_task->get_current_state() != PickTask::waiting) continue;
            std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > nT(new std::vector<std::shared_ptr<PickTask> >());
            std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > not_nT = PickTask::copy_picktasks_v2(*_pick_tasks_ref);
            nT->push_back(cur_task);
            std::shared_ptr<Graph> g_nT(new Graph(nT, _ce));
            PickTask::delete_picktasks_v2(not_nT, cur_task);
            std::shared_ptr<Graph> g_not_nT(new Graph(not_nT, _ce));
            int32_t dT = cur_task->get_demand();
            cur_task->set_current_state(PickTask::testing);
            //std::cout << i <<"_robot->get_current_dataset_capacity() = " << _robot->get_current_dataset_capacity() << std::endl;
            while(dT < cur_robot->get_current_dataset_capacity()) {
                //std::cout << "\tdT = " << dT << std::endl;
                std::shared_ptr<PickTask> best_feasible_task = g_not_nT->restricted_nearest_neighbor_v2(cur_task, dT, cur_robot);
                if (!best_feasible_task)
                    break;
                best_feasible_task->set_current_state(PickTask::testing);
                dT += best_feasible_task->get_demand();
                nT->push_back(best_feasible_task);
                PickTask::delete_picktasks_v2(not_nT, best_feasible_task);
                if (nT->size() == RSE::route_size) break;
            }
            g_not_nT.reset();
            g_nT.reset();
            g_nT = std::make_shared<Graph>(nT, _ce);
            g_not_nT = std::make_shared<Graph>(not_nT, _ce);
            //std::cout << "euler tour" << std::endl;
            auto ec_tpl = g_nT->euler_tour_v3(cur_robot);
            //std::cout << "single node cycle" << std::endl;
            auto snc_tpl = g_not_nT->single_node_cycle_v2(cur_robot);
            //std::cout << "ok" << std::endl;
            float p = std::get<1>(ec_tpl) + std::get<1>(snc_tpl);
            if (p <= p_min) {
            //if (std::get<1>(snc_tpl) < p_min && std::get<1>(ec_tpl) < best_cost) {
                //best_cost = std::get<1>(ec_tpl);
                p_min = p;
                t_min = nT;
                not_t_min = not_nT;
                best_robot = cur_robot;
                changed = true;
            }
            g_not_nT.reset();
            g_nT.reset();
        }
    }
    if (!changed) {
        fprintf(stderr, "Cannot find a route...\n");
        RSE::show<std::vector<std::shared_ptr<PickTask> > >(*_pick_tasks_ref);
        exit(1);
        /*t_min.reset();
        not_t_min.reset();
        return std::make_tuple(FLT_MAX, t_min, not_t_min);*/
    }

#ifdef DEBUG
    std::cout << "Returned t_min: ";
    for (int i = 0; i < t_min->size(); i++) {
        std::shared_ptr<Task> cur_task = (*t_min)[i];
        std::cout << "t" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
    std::cout << "Returned not_t_min: ";
    for (int i = 0; i < not_t_min->size(); i++) {
        std::shared_ptr<Task> cur_task = (*not_t_min)[i];
        std::cout << "t" << cur_task->get_id() << " ";
    }
    std::cout << std::endl;
#endif
    return std::make_tuple(p_min, t_min, not_t_min, best_robot);
}

std::vector<std::shared_ptr<Robot> > Scheduler::next_robots() {
#ifdef DEBUG
    std::cout << "Arrivals list (starting next_robots):" << std::endl;
    this->show_arrivals_v3();
    std::cout << std::endl;
#endif
    
    float min_time = FLT_MAX;
    int min_iteration = INT_MAX;
    std::vector<std::shared_ptr<Robot> > best_robots;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> cur_robot = std::get<0>(this->__arrivals[i]);
        if (cur_robot->get_state() == Robot::working) {
            float cur_time = std::get<1>(this->__arrivals[i]);
            int cur_iteration = std::get<3>(this->__arrivals[i]);
            if (cur_iteration < min_iteration) {
                min_iteration = cur_iteration;
                min_time = cur_time;
                best_robots.clear();
                best_robots.push_back(cur_robot);
            }
            else if (cur_iteration == min_iteration) {
                if (cur_time < min_time) {
                    min_time = cur_time;
                    best_robots.clear();
                    best_robots.push_back(cur_robot);
                }
                else if (cur_time == min_time) {
                    best_robots.push_back(cur_robot);
                }
            }
        }
    }
    
    for (int i = 0; i < this->__arrivals.size(); i++) {
        std::shared_ptr<Robot> rbt = std::get<0>(this->__arrivals[i]);
        float tm = std::get<1>(this->__arrivals[i]);
        float iteration = std::get<3>(this->__arrivals[i]);
        std::vector<std::shared_ptr<Robot> >::iterator it = std::find_if(best_robots.begin(), best_robots.end(), 
                [&rbt](std::shared_ptr<Robot> a1){ return a1->get_id() == rbt->get_id();} );
        if (it != best_robots.end()) { // encontrou
            this->__arrivals[i] = std::make_tuple(rbt, tm, 0, iteration + 1, false);
        }
    }
    
#ifdef DEBUG
    std::cout << "Arrivals list (ending next_robots):" << std::endl;
    this->show_arrivals_v3();
    std::cout << std::endl;
#endif
    
#ifdef DEBUG
    std::cout << "Returning robots: { ";
    for (int i = 0; i < best_robots.size(); i++)
        std::cout << "r" << best_robots[i]->get_id() << " ";
    std::cout << "}" << std::endl;
#endif
    
    return best_robots;
}

void Scheduler::update_dynamic_pos(std::vector<std::shared_ptr<Robot> > _robots, std::shared_ptr<CostEstimator> _ce) {
#ifdef DEBUG
    std::cout << "Arrivals list (v2):" << std::endl;
    this->show_arrivals_v3();
    std::cout << std::endl;
#endif
    
    //Primeiro, procura o endereço do robô em arrivals
    std::shared_ptr<Robot> rbt;
    float tm;
    for (int i = 0; i < this->__arrivals.size(); i++) {
        rbt = std::get<0>(this->__arrivals[i]);
        if (rbt->get_state() == Robot::working) {
            tm = std::get<1>(this->__arrivals[i]);
            break;
        }
    }
    
    //atualizando posição de todos os robôs
    int n_robots = this->__arrivals.size();
    for (int i = 0; i < n_robots; i++) {
        std::shared_ptr<Robot> cur_robot = std::get<0>(this->__arrivals[i]);
        std::shared_ptr<Task> last_task = cur_robot->get_last_task();
        if (cur_robot->get_state() != Robot::working || !last_task) continue;
        float cur_time = std::get<1>(this->__arrivals[i]);
        int cur_iteration = std::get<3>(this->__arrivals[i]);
#ifdef DEBUG
        std::cout << "    cur_robot: " << cur_robot->get_id() << std::endl;
        std::cout << "    last_task: " << last_task->get_id() << std::endl << std::endl;
#endif
        /*std::vector<std::shared_ptr<Robot> >::iterator it = std::find_if(_robots.begin(), _robots.end(), 
                [&cur_robot](std::shared_ptr<Robot> a1){ return a1->get_id() == cur_robot->get_id();} );*/
        //if (it != _robots.end()) { // encontrou
        if (cur_robot->get_id() == rbt->get_id()) {
            cur_robot->set_x(last_task->get_x());
            cur_robot->set_y(last_task->get_y());
            cur_robot->set_state(Robot::idle);
            float new_tm = 0.0;
            this->__arrivals[i] = std::make_tuple(cur_robot, new_tm, 0, cur_iteration, false);
        }
        else {
            float m, n;
            if (last_task->get_x() > cur_robot->get_x() && last_task->get_y() > cur_robot->get_y()) {
                m = 1.0;
                n = 1.0;
            }
            else if (last_task->get_x() > cur_robot->get_x() && last_task->get_y() == cur_robot->get_y()) {
                m = 1.0;
                n = 0.0;
            }
            else if (last_task->get_x() > cur_robot->get_x() && last_task->get_y() < cur_robot->get_y()) {
                m = 1.0;
                n = -1.0;
            }
            else if (last_task->get_x() == cur_robot->get_x() && last_task->get_y() < cur_robot->get_y()) {
                m = 0.0;
                n = -1.0;
            }
            else if (last_task->get_x() < cur_robot->get_x() && last_task->get_y() < cur_robot->get_y()) {
                m = -1.0;
                n = -1.0;
            }
            else if (last_task->get_x() < cur_robot->get_x() && last_task->get_y() == cur_robot->get_y()) {
                m = -1.0;
                n = 0.0;
            }
            else if (last_task->get_x() < cur_robot->get_x() && last_task->get_y() > cur_robot->get_y()) {
                m = -1.0;
                n = 1.0;
            }
            else if (last_task->get_x() == cur_robot->get_x() && last_task->get_y() > cur_robot->get_y()) {
                m = 0.0;
                n = 1.0;
            }
            float t = (cur_time - tm < 0) ? 0 : cur_time - tm;
#ifdef DEBUG
            std::cout << "    x_t: " << last_task->get_x() << ", y_t: " << last_task->get_y() << ", x_r: " << cur_robot->get_x() << ", y_r: " << cur_robot->get_y() <<
                    ", v_r: " << cur_robot->get_max_linear_speed() << ", t': " << t << ", m: " << m << ", n: " << n << std::endl;
#endif
            uint16_t new_x = (uint16_t)round(last_task->get_x() + (((last_task->get_x() - cur_robot->get_x()) * cur_robot->get_max_linear_speed() * t) / (((cur_robot->get_x() - last_task->get_x()) * m) + ((cur_robot->get_y() - last_task->get_y()) * n))));
            uint16_t new_y = (uint16_t)round(last_task->get_y() + (((last_task->get_y() - cur_robot->get_y()) * cur_robot->get_max_linear_speed() * t) / (((cur_robot->get_x() - last_task->get_x()) * m) + ((cur_robot->get_y() - last_task->get_y()) * n))));
            cur_robot->set_x(new_x);
            cur_robot->set_y(new_y);
            float new_tm = _ce->get_cost_v2(cur_robot, last_task, cur_robot);
            this->__arrivals[i] = std::make_tuple(cur_robot, new_tm, 0, cur_iteration, false);
            if (new_tm == 0.0)
                cur_robot->set_state(Robot::idle);
        }
    }
    
    
#ifdef DEBUG
    std::cout << "Arrivals list (v2):" << std::endl;
    this->show_arrivals_v3();
    std::cout << std::endl;
#endif
}
