/* 
 * File:   Robot.hpp
 * Author: geoso
 *
 * Created on 17 de Fevereiro de 2021, 00:29
 */

#ifndef ROBOT_HPP
#define	ROBOT_HPP

#include <vector>
#include <tuple>
#include <typeinfo>
#include "PickTask.hpp"
#include "DeliverTask.hpp"
#include "RechargeTask.hpp"

class Robot : public Entity {
public:
    typedef enum rstat { idle = 0x1BE, working = 0x301, recharging = 0x41A, determined_failure = 0x768 } robotstate_t;

    Robot() = delete;
    Robot(uint16_t);
    Robot(uint16_t, Robot::robotstate_t);
    Robot(std::shared_ptr<Robot>);
    std::pair<uint16_t, uint16_t> get_initial_position();
    uint16_t get_initial_x();
    uint16_t get_initial_y();
    void set_initial_position(std::pair<uint16_t, uint16_t> _position);
    void set_initial_x(uint16_t _x);
    void set_initial_y(uint16_t _y);
    void set_id(uint16_t);
    void set_name(std::string);
    void set_state(Robot::robotstate_t);
    void set_type(std::string);
    void set_length(float);
    void set_width(float);
    void set_height(float);
    void set_weight(float);
    void set_real_capacity(float);
    void set_dataset_capacity(float);
    void set_load_surface(float);
    void set_min_linear_speed(float);
    void set_max_linear_speed(float);
    void set_angular_speed(float);
    void set_min_lifting_speed(float);
    void set_max_lifting_speed(float);
    void set_min_descending_speed(float);
    void set_max_descending_speed(float);
    void set_max_lifting_height(uint16_t);
    void set_battery_load(float);
    void set_min_battery_runtime(float);
    void set_max_battery_runtime(float);
    void set_battery_recharge_time(float);
    void set_battery_life_cycles(uint16_t);
    void set_min_working_temperature(uint8_t);
    void set_max_working_temperature(uint8_t);
    void set_min_working_humidity(float);
    void set_max_working_humidity(float);
    void set_laser_scanners(bool);
    void set_proximity_sensors(bool);
    void set_3d_cameras(bool);
    void set_2d_cameras(bool);
    void set_rear_sonar(bool);
    void set_bumpers(bool);
    void set_speaker(bool);
    void set_scheduling_cost(float);
    void reset_position();
    void reset_current_capacity();
    void reset_current_battery_load();
    void add_use_case(std::string);
    void add_description(std::string);
    void add_task_ref(std::shared_ptr<Task>);
    void set_executing_task(std::shared_ptr<Task>);
    void add_scheduling_cost(float);
    void increase_on_current_capacity(float);
    void decrease_from_current_capacity(float);
    void decrease_from_current_battery_load(float);
    void decrease_from_scheduling_cost(float);
    void recount_current_capacity();
    uint16_t get_id();
    std::string get_name();
    Robot::robotstate_t get_state();
    std::string get_type();
    Task::task_t get_type_of_last_task();
    Task::task_t get_type_task(int);
    int get_n_use_cases();
    std::string get_use_case(int);
    std::string get_description();
    float get_length();
    float get_width();
    float get_height();
    std::tuple<float, float, float> get_dimensions();
    float get_weight();
    float get_real_capacity();
    float get_total_dataset_capacity();
    float get_load_surface();
    float get_min_linear_speed();
    float get_max_linear_speed();
    std::pair<float, float> get_linear_speed();
    float get_angular_speed();
    float get_min_lifting_speed();
    float get_max_lifting_speed();
    std::pair<float, float> get_lifting_speed();
    float get_min_descending_speed();
    float get_max_descending_speed();
    std::pair<float, float> get_descending_speed();
    uint16_t get_max_lifting_height();
    float get_battery_load();
    float get_min_battery_runtime();
    float get_max_battery_runtime();
    std::pair<float, float> get_battery_runtime();
    float get_battery_recharge_time();
    uint16_t get_battery_life_cycles();
    uint8_t get_min_working_temperature();
    uint8_t get_max_working_temperature();
    std::pair<uint8_t, uint8_t> get_working_temperature();
    float get_min_working_humidity();
    float get_max_working_humidity();
    std::pair<float, float> get_working_humidity();
    int get_n_sensors();
    bool has_laser_scanners();
    bool has_proximity_sensors();
    bool has_3d_cameras();
    bool has_2d_cameras();
    bool has_rear_sonar();
    bool has_bumpers();
    bool has_speaker();
    int get_n_tasks();
    std::vector<std::shared_ptr<Task>> get_tasks_ref();
    std::shared_ptr<Task> get_task_ref(int);
    std::shared_ptr<Task> get_last_task();
    std::shared_ptr<Task> get_executing_task();
    float get_scheduling_cost();
    float get_current_dataset_capacity();
    float get_current_battery_load();
    void remove_last_task();
    void show_me();
    void show_task_list();
    static bool is_it_here(std::vector<Robot*>, Robot*);
    static void show_robots(std::vector<Robot*>);
    static std::shared_ptr<std::vector<std::shared_ptr<Robot> > > clone_robots(std::vector<std::shared_ptr<Robot> >);
    ~Robot();
private:
    uint16_t __id;
    std::string __name;
    robotstate_t __state;
    std::pair<uint16_t, uint16_t> __initial_position;
    struct {
        std::string __type;
        std::vector<std::string> __use_cases;
        std::string __description;
        std::tuple<float, float, float> __dimensions; //length, width, height
        float __weight;
    } __general_specs;
    struct {
        float __real_capacity; // Robot specifications
        float __dataset_capacity; // Real capacity adapted to the dataset
        float __load_surface;
    } __load_specs;
    struct {
        std::pair<float, float> __linear_speed; //empty and loaded
        float __angular_speed;
    } __motion_specs;
    struct {
        std::pair<float, float> __lifting_speed; //empty and loaded
        std::pair<float, float> __descending_speed; //empty and loaded
        uint16_t __max_lifting_height;
    } __gripper_specs;
    struct {
        float __load;
        std::pair<float, float> __runtime_interval;
        float __recharge_time;
        uint16_t __life_cycles;
    } __battery_specs;
    struct {
        std::pair<uint8_t, uint8_t> __temperature_interval;
        std::pair<float, float> __humidity_interval;
    } __ambient_specs;
    struct {
        bool __laser_scanners;
        bool __proximity_sensors;
        bool __3d_cameras;
        bool __2d_cameras;
        bool __rear_sonar;
        bool __bumpers;
        bool __speaker;
    } __sensors;
    struct {
        float __cur_capacity;
        float __cur_battery_load;
    } __dyn_resources;
    struct {
        std::shared_ptr<Task> __executing_task;
        std::vector<std::shared_ptr<Task> > __tasks_ref;
        float __cost;
    } __scheduling;    
};

#endif	/* ROBOT_HPP */

