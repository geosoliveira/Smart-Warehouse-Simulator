/* 
 * File:   Domain.cpp
 * Author: geoso
 * 
 * Created on 17 de Fevereiro de 2021, 18:49
 */

#include "Domain.hpp"

bool Domain::__sortByCost(std::pair<std::shared_ptr<PickTask>, float> _a1, std::pair<std::shared_ptr<PickTask>, float> _a2) {
    return (_a1.second < _a2.second);
}

Domain::Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots_ref, std::shared_ptr<CostEstimator> _ce) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots, std::shared_ptr<CostEstimator> _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__ce_ref = _ce;
    this->__pick_tasks_ref = _pick_tasks_ref;
    this->__robots_ref = _robots_ref;
#ifdef DEBUG
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*this->__robots_ref);
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*this->__pick_tasks_ref);
    this->__ce_ref->show_me();
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots, std::shared_ptr<CostEstimator> _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
}

Domain::Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks_ref, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots_ref, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref, std::shared_ptr<CostEstimator> _ce) {
#ifdef DEBUG
    fprintf(stdout, "\nGETTING IN: Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref, std::shared_ptr<CostEstimator> _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
    this->__ce_ref = _ce;
    this->__pick_tasks_ref = _pick_tasks_ref;
    this->__robots_ref = _robots_ref;
    this->__deliver_tasks_ref = _deliver_tasks_ref;
#ifdef DEBUG
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*this->__robots_ref);
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*this->__pick_tasks_ref);
    RSE::show<std::vector<std::shared_ptr<DeliverTask> > >(*this->__deliver_tasks_ref);
    this->__ce_ref->show_me();
#endif
#ifdef DEBUG
    fprintf(stdout, "LEAVING OF: Domain(std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > _pick_tasks, std::shared_ptr<std::vector<std::shared_ptr<Robot> > > _robots, std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > _deliver_tasks_ref, std::shared_ptr<CostEstimator> _ce) [%s:%d]\n", __FILE__, __LINE__);
#endif
}

void Domain::clear_domain_lists() {
    for (auto it = this->__domain_list.begin(); it != this->__domain_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::pair<std::shared_ptr<Robot>, float> > tpl_robots = std::get<1>(tpl);
        tpl_robots.clear();
    }
    this->__domain_list.clear();
    
    for (auto it = this->__dominant_list.begin(); it != this->__dominant_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::pair<std::shared_ptr<PickTask>, float> > tpl_tasks = std::get<1>(tpl);
        tpl_tasks.clear();
    }
    this->__dominant_list.clear();
    
    for (auto it = this->__deliver_dominant_list.begin(); it != this->__deliver_dominant_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::pair<std::shared_ptr<PickTask>, float> > tpl_tasks = std::get<1>(tpl);
        tpl_tasks.clear();
    }
    this->__deliver_dominant_list.clear();
}

void Domain::compute_depot_domain() {  
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > best_pick_tasks;
    int n_pick_tasks = this->__pick_tasks_ref->size();
    int n_deliver_tasks = this->__deliver_tasks_ref->size();
    for (int i = 0; i < n_deliver_tasks; i++) {
        std::shared_ptr<DeliverTask> cur_deliver = (*this->__deliver_tasks_ref)[i];
        float best_cost = FLT_MAX;
        for (int j = 0; j < n_pick_tasks; j++) {
            std::shared_ptr<PickTask> cur_pick_task = (*this->__pick_tasks_ref)[j];
            float cur_cost;
            cur_cost = this->__ce_ref->get_manhattan_cost_v2(cur_deliver, cur_pick_task);
            if (cur_cost < best_cost) {
                best_cost = cur_cost;
                best_pick_tasks.clear();
                best_pick_tasks.push_back(std::make_pair(cur_pick_task, cur_cost));
            }
            else if (cur_cost == best_cost) {
                best_pick_tasks.push_back(std::make_pair(cur_pick_task, cur_cost));
            }
        }
        std::tuple<std::shared_ptr<DeliverTask>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > tpl = std::make_tuple(cur_deliver, best_pick_tasks);
        // Push in primary domain list
        this->__deliver_dominant_list.insert({cur_deliver->get_id(), tpl});
    }
}

void Domain::compute_domain_v1() {  
    std::vector<std::pair<std::shared_ptr<Robot>, float> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost;
                if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                    cur_cost = FLT_MAX;
                }
                else {
                    cur_cost = this->__ce_ref->get_cost_v2(cur_task, cur_robot, cur_robot);
                }
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(std::make_pair(cur_robot, cur_cost));
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(std::make_pair(cur_robot, cur_cost));
                }
            }
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > > tpl = std::make_tuple(cur_task, best_robots);
            // Push in primary domain list
            this->__domain_list.insert({cur_task->get_id(), tpl});
        }
    }
}

void Domain::compute_domain_v4() {
    std::vector<std::pair<std::shared_ptr<Robot>, float> > best_robots;
    int n_robots = this->__robots_ref->size();
    int n_tasks = this->__pick_tasks_ref->size();
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        float best_cost = FLT_MAX;
        if (cur_task->get_current_state() == PickTask::waiting) {
            for (int j = 0; j < n_robots; j++) {
                std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[j];
                float cur_cost = this->__ce_ref->get_sore_cost(cur_robot->get_position(), cur_task, cur_robot);
                /*int32_t capacity_after_pickup_i;
                if (cur_robot->get_total_dataset_capacity() < cur_task->get_demand()) {
                    cur_cost = FLT_MAX;
                }
                else {
                    if (cur_robot->get_current_dataset_capacity() >= cur_task->get_demand()) {
                        // Pode pegar
                        cur_cost = this->__ce_ref->get_cost_v2(cur_task, cur_robot, cur_robot);
                        capacity_after_pickup_i = cur_robot->get_current_dataset_capacity() - cur_task->get_demand();
                    }
                    else {
                        // Não pode pegar -- o custo é o custo de ir à estação de entrega mais próxima e, de lá, voltar à cur_task
                        auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_robot, cur_robot);
                        cur_cost = best_deliver_tuple.second;
                        cur_cost += this->__ce_ref->get_cost_v2(best_deliver_tuple.first, cur_task, cur_robot);
                        capacity_after_pickup_i = cur_robot->get_total_dataset_capacity() - cur_task->get_demand();
                    }
                    // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
                    // Verificando se o robô ficará impedido de pegar outra tarefa:
                    bool can_exec_another_task = false;
                    for (int k = 0; k < n_tasks; k++) {
                        std::shared_ptr<PickTask> cur_task_of_all = (*this->__pick_tasks_ref)[k];
                        if (cur_task->get_id() != cur_task_of_all->get_id() &&
                                cur_task_of_all->get_current_state() == PickTask::waiting &&
                                capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                            can_exec_another_task = true;
                            break;
                        }
                    }
                    // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
                    if (!can_exec_another_task) {
                        auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_task, cur_robot);
                        cur_cost += best_deliver_tuple.second;
                    }
                }*/
                if (cur_cost < best_cost) {
                    best_cost = cur_cost;
                    best_robots.clear();
                    best_robots.push_back(std::make_pair(cur_robot, cur_cost));
                }
                else if (cur_cost == best_cost) {
                    best_robots.push_back(std::make_pair(cur_robot, cur_cost));
                }
            }
            // Push in domain list
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > > tpl_task = std::make_tuple(cur_task, best_robots);
            this->__domain_list.insert({cur_task->get_id(), tpl_task});
            /*for (int j = 0; j < best_robots.size(); j++) {
                std::vector<std::shared_ptr<PickTask> > *tasks_of_this_robot = &(std::get<1>(this->__dominant_list[best_robots[j]->get_id()]));
                std::vector<std::shared_ptr<PickTask> >& tasks_of_this_robot_deref = *(tasks_of_this_robot);
                tasks_of_this_robot_deref.push_back(cur_task);
                std::tuple<std::shared_ptr<Robot>, std::vector<std::shared_ptr<PickTask> > > tpl_robot = std::make_tuple(best_robots[j], tasks_of_this_robot_deref);
                this->__dominant_list.insert({best_robots[j]->get_id(), tpl_robot});
            }*/
        }
    }
}

void Domain::compute_dominants() {
    int n_tasks = this->__pick_tasks_ref->size();
    int n_robots = this->__robots_ref->size();
    for (int i = 0; i < n_robots; i++) {
        std::vector<std::pair<std::shared_ptr<PickTask>, float> > temp_tasks;
        std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > tpl = std::make_tuple((*this->__robots_ref)[i], temp_tasks);
        this->__dominant_list.insert({(*this->__robots_ref)[i]->get_id(), tpl});
    }
    for (int i = 0; i < n_tasks; i++) {
        std::shared_ptr<PickTask> cur_task = (*this->__pick_tasks_ref)[i];
        if (cur_task->get_current_state() == PickTask::waiting) {
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > >
                    tpl_list = this->__domain_list[cur_task->get_id()];
            std::vector<std::pair<std::shared_ptr<Robot>, float> > cur_robot_list = std::get<1>(tpl_list);
            for (int j = 0; j < cur_robot_list.size(); j++) {
                std::shared_ptr<Robot> cur_robot = cur_robot_list[j].first;
                float cur_cost = cur_robot_list[j].second;
                std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
                        tpl_trasnp_list = this->__dominant_list[cur_robot->get_id()];
                std::vector<std::pair<std::shared_ptr<PickTask>, float> > cur_task_list = std::get<1>(tpl_trasnp_list);
                cur_task_list.push_back(std::make_pair(cur_task, cur_cost));
                std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
                        tpl = std::make_tuple(cur_robot, cur_task_list);
                this->__dominant_list[cur_robot->get_id()] = tpl;
            }
        }
    }
}

void Domain::remove_ptask_from_dominants(std::shared_ptr<PickTask> _ptask, std::shared_ptr<Robot> _robot) {
    std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > > tpl_domain_list = this->__domain_list[_ptask->get_id()];
    std::vector<std::pair<std::shared_ptr<Robot>, float> > robots_from_ptask = std::get<1>(tpl_domain_list);
    // Se _ptask é dominada por vários robôs, então _robots_from_ptask.size() > 1
    float cost_from_ptask_to_robots = robots_from_ptask[0].second; // O custo é o mesmo para todos
    robots_from_ptask.clear();
    robots_from_ptask.push_back(std::make_pair(_robot, cost_from_ptask_to_robots));
    tpl_domain_list = std::make_tuple(_ptask, robots_from_ptask);
    this->__domain_list[_ptask->get_id()] = tpl_domain_list;
        
    int n_robots = this->__robots_ref->size();
    for (int i = 0; i < n_robots; i++) {
        std::shared_ptr<Robot> cur_robot = (*this->__robots_ref)[i];
        if (cur_robot->get_id() != _robot->get_id()){
            std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > tpl_dominant_list = this->__dominant_list[cur_robot->get_id()];
            std::vector<std::pair<std::shared_ptr<PickTask>, float> > ptasks_from_cur_robot = std::get<1>(tpl_dominant_list);
            std::vector<std::pair<std::shared_ptr<PickTask>, float> >:: iterator it;
            for (it = ptasks_from_cur_robot.begin(); it != ptasks_from_cur_robot.end(); ++it) {
                if (it->first == _ptask) {
                    ptasks_from_cur_robot.erase(it);
                    tpl_dominant_list = std::make_tuple(cur_robot, ptasks_from_cur_robot);
                    this->__dominant_list[cur_robot->get_id()] = tpl_dominant_list;
                    break;
                }
            }
        }
    }
}

std::vector<std::shared_ptr<PickTask> > Domain::get_domain_of_robot(std::shared_ptr<Robot> _robot) {
     std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > all_tasks = std::get<1>(tpl);
    //Retuning waiting tasks
    std::vector<std::shared_ptr<PickTask> > waiting_tasks;
    for (int i = 0; i < all_tasks.size(); i++) {
        if (all_tasks[i].first->get_current_state() == PickTask::waiting) {
            waiting_tasks.push_back(all_tasks[i].first);
        }
    }
    return waiting_tasks;
}

std::vector<std::pair<std::shared_ptr<PickTask>, float> > Domain::get_domain_of_robot_full(std::shared_ptr<Robot> _robot) {
     std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > all_tasks = std::get<1>(tpl);
    //Retuning waiting tasks
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > resulting_tasks;
    for (int i = 0; i < all_tasks.size(); i++) {
        if (all_tasks[i].first->get_current_state() == PickTask::waiting) {
            resulting_tasks.push_back(std::make_pair(all_tasks[i].first, all_tasks[i].second));
        }
    }
    return resulting_tasks;
}

std::vector<std::shared_ptr<Robot> > Domain::get_dominants_of_task(std::shared_ptr<PickTask> _p_task) {
    std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > >
            tpl = this->__domain_list[_p_task->get_id()];
    
    std::vector<std::pair<std::shared_ptr<Robot>, float> > all_robots = std::get<1>(tpl);
    //Retuning robots
    std::vector<std::shared_ptr<Robot> > returning_robots;
    for (int i = 0; i < all_robots.size(); i++) {
        returning_robots.push_back(all_robots[i].first);
    }
    return returning_robots;
}

std::vector<std::pair<std::shared_ptr<Robot>, float> > Domain::get_dominants_of_task_full(std::shared_ptr<PickTask> _p_task) {
     std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > >
            tpl = this->__domain_list[_p_task->get_id()];
    std::vector<std::pair<std::shared_ptr<Robot>, float> > all_robots = std::get<1>(tpl);
    //Retuning waiting tasks
    std::vector<std::pair<std::shared_ptr<Robot>, float> > resulting_robots;
    for (int i = 0; i < all_robots.size(); i++) {
        resulting_robots.push_back(std::make_pair(all_robots[i].first, all_robots[i].second));
    }
    return resulting_robots;
}

std::vector<std::shared_ptr<PickTask> > Domain::get_domain_of_depot(std::shared_ptr<DeliverTask> _deliver) {
     std::tuple<std::shared_ptr<DeliverTask>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
            tpl = this->__deliver_dominant_list[_deliver->get_id()];
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > all_tasks = std::get<1>(tpl);
    //Retuning waiting tasks
    std::vector<std::shared_ptr<PickTask> > waiting_tasks;
    for (int i = 0; i < all_tasks.size(); i++) {
        if (all_tasks[i].first->get_current_state() == PickTask::waiting) {
            waiting_tasks.push_back(all_tasks[i].first);
        }
    }
    return waiting_tasks;
}

int Domain::get_n_primary_dominants_by_r_v2(std::shared_ptr<Robot> _robot) {
    std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > cur_tasks_ref = std::get<1>(tpl);
    return cur_tasks_ref.size();
}

std::map<uint16_t, std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > > >
Domain::get_dominants() {
    return this->__dominant_list;
}

std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
Domain::get_tuple_of_robot(uint16_t _id) {
    return this->__dominant_list[_id];
}

std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > >
Domain::get_tuple_of_task(uint16_t _id) {
    return this->__domain_list[_id];
}

void Domain::show_domain_of_robot(std::shared_ptr<Robot> _robot) {
    std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
            tpl = this->__dominant_list[_robot->get_id()];
    std::vector<std::pair<std::shared_ptr<PickTask>, float> > cur_tasks = std::get<1>(tpl);
    std::cout << "Robot " << _robot->get_id() << " (x = " << _robot->get_x() <<
            ", y = " << _robot->get_y() << ", capacity = " << _robot->get_current_dataset_capacity() <<
            ") domains over tasks:" << std::endl;
    for (int j = 0; j < cur_tasks.size(); j++) {
        std::cout << "\tt" << cur_tasks[j].first->get_id() << ". Cost = " << cur_tasks[j].second << std::endl;
    }
}

void Domain::show_domains() {
    std::cout << "Domain list:" << std::endl;
    for (int i = 0; i < this->__pick_tasks_ref->size(); i++) {
        std::shared_ptr<PickTask> cur_task_at_domain_list = (*this->__pick_tasks_ref)[i];
        if (cur_task_at_domain_list->get_current_state() == PickTask::waiting) {
            std::tuple<std::shared_ptr<PickTask>, std::vector<std::pair<std::shared_ptr<Robot>, float> > >
                    tpl = this->__domain_list[cur_task_at_domain_list->get_id()];
            std::vector<std::pair<std::shared_ptr<Robot>, float> > cur_robots_at_domain_list = std::get<1>(tpl);
            std::cout << "Task " << cur_task_at_domain_list->get_id() << " (x = " << cur_task_at_domain_list->get_x() <<
                ", y = " << cur_task_at_domain_list->get_y() << ", demand = " << cur_task_at_domain_list->get_demand() <<
                ") dominated by robots:" << std::endl;
            for (int j = 0; j < cur_robots_at_domain_list.size(); j++) {
                std::cout << "\tr" << cur_robots_at_domain_list[j].first->get_id() << ". Cost = " << cur_robots_at_domain_list[j].second << std::endl;
            }
        }
    }
    std::cout << std::endl;
}

void Domain::show_dominants() {
    std::cout << "Dominant list:" << std::endl;
    for (int i = 0; i < this->__robots_ref->size(); i++) {
        std::shared_ptr<Robot> cur_robot_at_transposed_list = (*this->__robots_ref)[i];
        std::tuple<std::shared_ptr<Robot>, std::vector<std::pair<std::shared_ptr<PickTask>, float> > >
                tpl = this->__dominant_list[cur_robot_at_transposed_list->get_id()];
        std::vector<std::pair<std::shared_ptr<PickTask>, float> > cur_tasks_at_transposed_list = std::get<1>(tpl);            
        std::cout << "Robot " << cur_robot_at_transposed_list->get_id() << " (x = " << cur_robot_at_transposed_list->get_x() <<
                ", y = " << cur_robot_at_transposed_list->get_y() << ", capacity = " << cur_robot_at_transposed_list->get_current_dataset_capacity() <<
                ") domains over tasks:" << std::endl;
        for (int j = 0; j < cur_tasks_at_transposed_list.size(); j++) {
            std::cout << "\tt" << cur_tasks_at_transposed_list[j].first->get_id() << ". Cost = " << cur_tasks_at_transposed_list[j].second << std::endl;
        }
    }
    std::cout << std::endl;
}

Domain::~Domain() {
    for (auto it = this->__domain_list.begin(); it != this->__domain_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::pair<std::shared_ptr<Robot>, float> > tpl_robots = std::get<1>(tpl);
        tpl_robots.clear();
    }
    this->__domain_list.clear();
    
    for (auto it = this->__dominant_list.begin(); it != this->__dominant_list.end(); it++) {
        auto tpl = it->second;
        std::vector<std::pair<std::shared_ptr<PickTask>, float> > tpl_tasks = std::get<1>(tpl);
        tpl_tasks.clear();
    }
    this->__dominant_list.clear();
}

std::shared_ptr<PickTask>
Domain::cost_comparison(std::shared_ptr<Robot> _robot, std::vector<std::pair<std::shared_ptr<PickTask>, float> > _tasks) {
    
    std::shared_ptr<PickTask> best_task = NULL;
    float best_cost = floor(FLT_MAX / (1 + RSE::cost_tolerance));
    
    for (int i = 0; i < _tasks.size(); i++) {
        std::shared_ptr<PickTask> cur_task_of_main = _tasks[i].first;
        if (cur_task_of_main->get_current_state() == PickTask::waiting) {
#ifdef DEBUG
            std::cout << "\tIf r" << _robot->get_id() << " pick up t" << cur_task_of_main->get_id() << ": ";
#endif
            float cur_cost = this->__ce_ref->get_sore_cost(_robot->get_position(), cur_task_of_main, _robot);
#ifdef DEBUG
            std::cout << cur_cost;
#endif
            // Se ao pegar a próxima tarefa, o robô ficar impedido de pegar outra, então o custo é somado com o custo de voltar à estação de entrega
            // Verificando se o robô ficará impedido de pegar outra tarefa:
            /*bool can_exec_another_task = false;
            int32_t capacity_after_pickup_i = _robot->get_current_dataset_capacity() - cur_task_of_main->get_demand();
            for (int j = 0; j < this->__pick_tasks_ref->size(); j++) {
                std::shared_ptr<PickTask> cur_task_of_all = (*this->__pick_tasks_ref)[j];
                if (cur_task_of_main->get_id() != cur_task_of_all->get_id() &&
                        cur_task_of_all->get_current_state() == PickTask::waiting &&
                        capacity_after_pickup_i >= cur_task_of_all->get_demand()) {
                    can_exec_another_task = true;
                    break;
                }
            }
            // Verificou. Agora, se 'can_exec_another_task == false', incrementa o custo
            if (!can_exec_another_task) {
                auto best_deliver_tuple = this->__ce_ref->closer_deliver_v2(cur_task_of_main, _robot);
                cur_cost += best_deliver_tuple.second;
#ifdef DEBUG
            std::cout << " + " << best_deliver_tuple.second << std::endl;
#endif
            }*/
#ifdef DEBUG
            else {
                std::cout << std::endl;
            }
#endif

            // Comparando se o custo calculado é menor que o melhor custo conhecido
            if (cur_cost < best_cost * (1 - RSE::cost_tolerance)) {
                best_cost = cur_cost;
                best_task = cur_task_of_main;
            }
        }
    }
    
#ifdef DEBUG
    std::cout << "\tWinner Tasks: ";
    for (auto it = std::begin(*best_tasks_ref); it != std::end(*best_tasks_ref); ++it)
        std::cout << "t" << (*it)->get_id() << " ";
    std::cout << std::endl;
#endif
    return best_task;
}

std::shared_ptr<PickTask>
Domain::cost_comparison_v2(std::shared_ptr<Robot> _robot, std::vector<std::pair<std::shared_ptr<PickTask>, float> > _tasks) {
    std::sort(_tasks.begin(), _tasks.end(), Domain::__sortByCost);
    
#ifdef DEBUG
    std::cout << "\tWinner Task: ";
        std::cout << "t" << _tasks[0].first->get_id() << " ";
#endif
    return _tasks[0].first;
}
