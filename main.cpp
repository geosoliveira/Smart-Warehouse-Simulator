/* 
 * File:   main.cpp
 * Author: geoso
 *
 * Created on 16 de Fevereiro de 2021, 21:13
 */

#include <cstdlib>
#include <chrono> 
#include "Parser.hpp"
#include "Scheduler.hpp"
#include "Graph.hpp"

using namespace std;

int RSE::current_execution;
int RSE::number_executions;
float RSE::cost_tolerance;
int RSE::route_size;
std::vector<int> RSE::variable_tasks_at_time;
float RSE::map_scale;
unsigned RSE::seed;

int main(int argc, char** argv) {
    /* Normal use...*/
    /* Normal use...*/
    if (argc < 3 || argc > 5) {
        std::cerr << "Usage: " << argv[0] << " experiment <dataset_filename> [0:greedy|1:nCAR-v2|2:done-cpta|3:dyn-done-cpta|4:dyn-done-cpta-v2] <route size (nCAR only)>." << std::endl;
        std::cerr << "       " << argv[0] << " simulate warehouse <dataset_filename>" << std::endl;
        std::cerr << "       " << argv[0] << " simulate [order-generator|task-generator|robots]" << std::endl;
        std::cerr << "       " << argv[0] << " simulate scheduler [0:greedy|1:nCAR-v2|2:done-cpta|3:dyn-done-cpta|4:dyn-done-cpta-v2]" << std::endl;
        exit(1);
    }
    if (argc != 5) {
        std::string argv1 = argv[1];
        std::string argv2 = argv[2];
        if (argv1 != "simulate") {
            std::cerr << "Usage: " << argv[0] << " experiment <dataset_filename> [0:greedy|1:nCAR-v2|2:done-cpta|3:dyn-done-cpta|4:dyn-done-cpta-v2] <route size (nCAR only)>." << std::endl;
            std::cerr << "       " << argv[0] << " simulate warehouse <dataset_filename>" << std::endl;
            std::cerr << "       " << argv[0] << " simulate [order-generator|task-generator|robots]" << std::endl;
            std::cerr << "       " << argv[0] << " simulate scheduler [0:greedy|1:nCAR-v2|2:done-cpta|3:dyn-done-cpta|4:dyn-done-cpta-v2]" << std::endl;
            exit(1);
        }
        if (argv2 == "warehouse") {
            
        }
        else if (argv2 == "order-generator") {
            
        } 
        else if (argv2 == "task-generator") {
            
        } 
        else if (argv2 == "robots") {
            
        } 
        else if (argv2 == "scheduler") {
            
        }
        else {
            
        }
        return EXIT_SUCCESS;
    }
    
    std::string argv1 = argv[1];
    if (argv1 != "experiment") {
        std::cerr << "Usage: " << argv[0] << " experiment <dataset_filename> [0:greedy|1:nCAR-v2|2:done-cpta|3:dyn-done-cpta|4:dyn-done-cpta-v2] <route size (nCAR only)>." << std::endl;
        std::cerr << "       " << argv[0] << " simulate warehouse <dataset_filename>" << std::endl;
        std::cerr << "       " << argv[0] << " simulate [order-generator|task-generator|robots]" << std::endl;
        std::cerr << "       " << argv[0] << " simulate scheduler [0:greedy|1:nCAR-v2|2:done-cpta|3:dyn-done-cpta|4:dyn-done-cpta-v2]" << std::endl;
        exit(1);
    }
    
    auto entities = RSE::Parser::parse_file_v2(argv[2]);
    
    typedef std::chrono::high_resolution_clock myclock;
    myclock::time_point beginning = myclock::now();
    
    // obtain a seed from the timer
    myclock::duration d = myclock::now() - beginning;
    RSE::seed = d.count();
    
    RSE::cost_tolerance = 0.0;
    RSE::map_scale = 0.5;
    RSE::route_size = atoi(argv[4]);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > shared_pick_tasks = std::get<0>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > shared_deliver_tasks = std::get<1>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > shared_recharge_tasks = std::get<2>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > shared_robots = std::get<3>(entities);
    CostEstimator::costing_t costing_type = std::get<4>(entities);
    std::string dataset_name = std::get<5>(entities);
    
    std::map<int, std::vector<std::shared_ptr<PickTask> > > orders;
    for (int i = 0; i < shared_pick_tasks->size(); i++) {
        std::shared_ptr<PickTask> cur_task = (*shared_pick_tasks)[i];
        uint16_t order_id = cur_task->get_temp_order();
        orders[order_id].push_back(cur_task);
    }

#ifdef DEBUG
    for (auto it = orders.begin(); it != orders.end(); it++) {
        uint16_t order_id = it->first;
        fprintf(stdout, "Order %d:\n", order_id);
        std::vector<std::shared_ptr<PickTask> > tasks = it->second;
        for (int i = 0; i < tasks.size(); i++) {
            tasks[i]->show_me();
        }
        fprintf(stdout, "\n");
    }
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    //RSE::show<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::show<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    //RSE::show<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
#endif
    
    ofstream out_stream_results;
    out_stream_results.open("results.txt", std::ofstream::out | std::ofstream::app);

    if(!out_stream_results){
        fprintf(stderr, "Can't open output file results.txt. Exiting...\n");
        exit(1);
    }
    
    static std::tuple<int, int, float, std::vector<float> > tpl_sch;
    
    //auto start = std::chrono::high_resolution_clock::now();
    switch(atoi(argv[3])) {
        case 0:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' Greedy..." << std::endl;
            tpl_sch = Scheduler::greedy(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 1:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by nCAR (v2)..." << std::endl;
            tpl_sch = Scheduler::nCAR_v2(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 2:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Fixed DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::fixed_done_cpta(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 3:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Dyn DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::dyn_done_cpta(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        case 4:
            std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Dyn DoNe-CPTA (v2) with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
            tpl_sch = Scheduler::dyn_done_cpta_v2(shared_pick_tasks, shared_deliver_tasks, shared_recharge_tasks, shared_robots, costing_type);
            break;
        default:
            exit(1);
    }
    //auto end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    vector<float> it_durations = std::get<3>(tpl_sch);
    float duration = 0.0;
    for (int i = 0; i < it_durations.size(); i++) {
        duration += it_durations[i];
    }
    
    for (int i = 0; i < shared_robots->size(); i++) {
        if ((*shared_robots)[i]->get_n_tasks())
            (*shared_robots)[i]->show_task_list();
    }
    
    ofstream out_stream_dataset_orders;
    out_stream_dataset_orders.open(dataset_name + "." + argv[3] + ".res", std::ofstream::out | std::ofstream::app);

    if(!out_stream_dataset_orders){
        std::cerr << "Can't open output file " << dataset_name << ".txt. Exiting..." << std::endl;
        exit(1);
    }
    
    float sum = 0.0;
    for (auto it = orders.begin(); it != orders.end(); it++) {
        uint16_t order_id = it->first;
        std::vector<std::shared_ptr<PickTask> > tasks = it->second;
        float max_cost = 0.0;
        for (int i = 0; i < tasks.size(); i++) {
            std::shared_ptr<PickTask> cur_task = tasks[i];
            bool found = false;
            for (int j = 0; j < shared_robots->size(); j++) {
                std::shared_ptr<Robot> cur_robot = (*shared_robots)[j];
                std::vector<std::shared_ptr<Task> > tasks_this_robot = cur_robot->get_tasks_ref();
                for (int k = 0; k < tasks_this_robot.size(); k++) {
                    if (tasks_this_robot[k]->get_task_type() == Task::pick && cur_task->get_id() == tasks_this_robot[k]->get_id()) {
                        float cur_cost = cur_robot->get_scheduling_cost();
                        if (max_cost < cur_cost) max_cost = cur_cost;
                        found = true;
                        break;
                    }
                }
                if (found) break;
            }
        }
        out_stream_dataset_orders << max_cost << std::endl;
        sum += max_cost;
    }
    out_stream_dataset_orders.close();
    
    std::cout << "# Robots " << std::get<0>(tpl_sch) << std::endl;
    std::cout << "# Deliveries " << std::get<1>(tpl_sch) << std::endl;
    std::cout << "Cost " << std::get<2>(tpl_sch) << std::endl;
    std::cout << "Scheduling time " << duration << std::endl;
    std::cout << "Average time to fulfill orders " << sum / float(orders.size()) << std::endl;
    std::cerr << "For orders please see '" << dataset_name << "." << argv[3] << ".res'." << std::endl;
    
    out_stream_results << atoi(argv[3]) << "\t" << dataset_name.c_str() << "\t";
    out_stream_results << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << duration << "\t" << sum / float(orders.size()) << std::endl;
    out_stream_results.close();
    
    //RSE::reset_all<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    //shared_recharge_tasks.reset();
    shared_deliver_tasks.reset();
    shared_pick_tasks.reset();
    shared_robots.reset();
    
    return EXIT_SUCCESS;
    
    
    
    
    
    /* Benchmarking...*/
    /*if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <number_of_executions> <dataset_filename>" << std::endl;
        exit(1);
    }
    
    RSE::number_executions = atoi(argv[1]);
    RSE::cost_tolerance = 0.0;
    
    auto entities = RSE::Parser::parse_file(argv[2]);
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > shared_pick_tasks = std::get<0>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<DeliverTask> > > shared_deliver_tasks = std::get<1>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<RechargeTask> > > shared_recharge_tasks = std::get<2>(entities);
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > shared_robots = std::get<3>(entities);
    CostEstimator::costing_t costing_type = std::get<4>(entities);
    std::string dataset_name = std::get<5>(entities);
    
#ifdef DEBUG  
    RSE::show<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    RSE::show<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::show<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    //RSE::show<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
#endif
    
    static std::tuple<int, int, float, int> tpl_sch;
    double total_sch_time;
    
    ofstream out_stream;
    out_stream.open("results.txt", std::ofstream::out | std::ofstream::app);

    if(!out_stream){
        fprintf(stderr, "Can't open output file results.txt. Exiting...\n");
        exit(1);
    }
    
    out_stream << dataset_name.c_str() << "\t";
    
    std::shared_ptr<std::vector<std::shared_ptr<PickTask> > > tasks_cp;
    std::shared_ptr<std::vector<std::shared_ptr<Robot> > > robots_cp;
    */
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by FIFO... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::fifo(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << std::endl;
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Greedy... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::greedy(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << std::endl;
    
    RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by nCAR... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::nCAR_v2(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << std::endl;
    */
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::done_cpta(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << std::endl;
    */
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Dyn DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::dyn_done_cpta(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << std::endl;
    */
    /*RSE::current_execution = 1;
    total_sch_time = 0.0;
    std::cerr << "Performing dataset '" << dataset_name.c_str() << "' by Fixed DoNe-CPTA with cost tolerance " << RSE::cost_tolerance << "... " << std::endl;
    for (int i = 0; i < RSE::number_executions; i++) {
        tasks_cp = PickTask::clone_picktasks(*shared_pick_tasks);
        robots_cp = Robot::clone_robots(*shared_robots);
        auto start = std::chrono::high_resolution_clock::now();
        tpl_sch = Scheduler::fixed_done_cpta(tasks_cp, shared_deliver_tasks, shared_recharge_tasks, robots_cp, costing_type);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        total_sch_time += duration.count() / 1000000.0;
        RSE::current_execution++;
    }
    out_stream << std::get<0>(tpl_sch) << "\t" << std::get<1>(tpl_sch) << "\t" << std::get<2>(tpl_sch) << "\t" << (total_sch_time / ((double)RSE::number_executions)) << std::endl;
    */
    /*out_stream.close();
    
    //RSE::reset_all<std::vector<std::shared_ptr<RechargeTask> > >(*shared_recharge_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<DeliverTask> > >(*shared_deliver_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<PickTask> > >(*shared_pick_tasks);
    RSE::reset_all<std::vector<std::shared_ptr<Robot> > >(*shared_robots);
    //shared_recharge_tasks.reset();
    shared_deliver_tasks.reset();
    shared_pick_tasks.reset();
    shared_robots.reset();
    
    return EXIT_SUCCESS;*/
}

