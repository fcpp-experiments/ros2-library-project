// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file ap_engine_setup.hpp
 * @brief Setup for the AP engine.
 */

#ifndef NODES_AP_ENGINE_SETUP_H_
#define NODES_AP_ENGINE_SETUP_H_

#include <cmath>
#include <chrono>
#include <ctime>
#include "lib/poc_config.hpp"
#include "lib/fcpp.hpp"

/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

using namespace std::chrono;

//! @cond INTERNAL
namespace coordination {
    struct main;   // forward declaration of main function
    struct main_t; // forward declaration of main exports
}
//! @endcond


namespace coordination {
    namespace tags {
    //! @brief The average round interval of a device.
    struct tavg {};

    //! @brief The variance of round timing in the network.
    struct tvar {};

    //! @brief The number of devices.
    struct devices {};

    //! @brief The movement speed of devices.
    struct speed {};

    //! @brief The horizontal side of deployment area.
    struct sidex {};

    //! @brief The vertical side of deployment area.
    struct sidey {};

    //! @brief Color of the current node.
    struct node_color {};

    //! @brief Left color of the current node.
    struct left_color {};

    //! @brief Right color of the current node.
    struct right_color {};

    //! @brief Size of the current node.
    struct node_size {};

    //! @brief Shape of the current node.
    struct node_shape {};

    //! @brief Size of the label of the current node.
    struct node_label_size {};

    //! @brief Text of the label of the current node.
    struct node_label_text {};

    //! @brief Color of the label of the current node.
    struct node_label_color {};

    //! @brief Shape of the shadow of current node.
    struct node_shadow_shape {};

    //! @brief Size of the shadow of the current node.
    struct node_shadow_size {};

    //! @brief Color of the shadow of the current node.
    struct node_shadow_color {};

    //! @brief Status of the current node's goal read by robot feedback.
    struct node_ext_goal_status {};

    //! @brief Current step of goal
    struct node_ext_goal_current_step {};

    //! @brief Status of the current node's docking read by robot feedback.
    struct node_ext_docking_status {};

    //! @brief Status of the current node's system read by robot feedback.
    struct node_ext_system_status {};

    //! @brief Informations read from robot feedback for each goal.
    struct node_ext_goal_update_info {};

    //! @brief Timestamp of the updated goal read from robot feedback.
    struct node_ext_goal_update_time {};

    //! @brief Goal of the current node read by robot feedback. 
    struct node_ext_goal {};

    //! @brief External name of a node used by applications (different from AP).
    struct node_ext_name {};

    //! @brief Status of the AP process of the current node.
    struct node_process_status {};

    //! @brief Goal of the current node in AP process.
    struct node_process_goal {};

    //! @brief List of computing goals of the current node in AP process.
    struct node_process_computing_goals {};

    //! @brief Battery charge of the current node.
    struct node_battery_charge {};

    //! @brief Offset of "x" position of the node
    struct node_offset_pos_x {};

    //! @brief Offset of "y" position of the node
    struct node_offset_pos_y {};

    //! @brief Action of the goal
    struct goal_action {};

    //! @brief Code of the goal
    struct goal_code {};

    //! @brief Position X of the goal
    struct goal_pos_start_x {};

    //! @brief Position Y of the goal
    struct goal_pos_start_y {};

    //! @brief Orient W of the goal
    struct goal_orient_start_w {};

    //! @brief Position X of the goal
    struct goal_pos_end_x {};

    //! @brief Position Y of the goal
    struct goal_pos_end_y {};

    //! @brief Orient W of the goal
    struct goal_orient_end_w {};

    //! @brief Source of the goal
    struct goal_source {};

    //! @brief Priority of the goal
    struct goal_priority {};

    //! @brief Subcode of the goal
    struct goal_subcode {};

    //! @brief Step of the goal
    struct goal_step {};
} // tags

    //! @brief Communication radius.
    constexpr size_t comm = AP_COMM_RANGE;

    //! @brief Retain time for messages.
    constexpr size_t retain = AP_RETAIN_SEC;

    //! @brief Dimensionality of the space.
    constexpr size_t dim = 3;

    //! @brief Min area of the rectangle.
    const vec<3> amin = make_vec(0.0,0.0,0.0);

    //! @brief Max area of the rectangle.
    const vec<3> amax = make_vec(AXIS_X_LENGTH,AXIS_Y_LENGTH,0.0);

    //! @brief Diameter of nodes graph.
    const int graph_diameter = (CUSTOM_GRAPH_DIAMETER != NULL_INT_VALUE) 
                                    ? CUSTOM_GRAPH_DIAMETER
                                    // 2 * min(n_robots, 2 * (round of max_distance / communication range))
                                    : 2*static_cast<int>(std::min(ROBOTS_COUNT*1.0, 2.0*ceil(norm(amax - amin)/comm)));

    // simulator vars
    constexpr vec<2> computing_colors  = make_vec(0xADAD00FF, fcpp::YELLOW);

    constexpr fcpp::packed_color idle_color         = BLACK;
    constexpr fcpp::packed_color illegal_color      = BLUE;
    constexpr fcpp::packed_color running_color      = ORANGE;
    constexpr fcpp::packed_color docking_color      = GRAY;
    constexpr fcpp::packed_color reached_goal_color = GREEN;
    constexpr fcpp::packed_color failed_goal_color  = TOMATO;
    constexpr fcpp::packed_color aborted_goal_color = RED;
    constexpr fcpp::packed_color discharged_color   = MAROON;
}

//! @brief Namespace for component options.
namespace option {

//! @brief Import tags to be used for component options.
using namespace component::tags;
//! @brief Import tags used by aggregate functions.
using namespace coordination::tags;

//! @brief Maximum admissible value for a seed.
constexpr size_t seed_max = std::min<uintmax_t>(std::numeric_limits<uint_fast32_t>::max(), std::numeric_limits<intmax_t>::max());

//! @brief Shorthand for a constant numeric distribution.
template <intmax_t num, intmax_t den = 1>
using n = distribution::constant_n<double, num, den>;

//! @brief Shorthand for a uniform numeric distribution.
template <intmax_t max, intmax_t min=0>
using nu = distribution::interval_n<double, min, max>;

//! @brief Shorthand for an constant input distribution.
template <typename T, typename R = double>
using i = distribution::constant_i<R, T>;

//! @brief Shorthand for an constant input distribution.
template <typename T, typename R = std::unordered_map<std::string, std::vector<device_t>>>
using subcode_map_distr = distribution::constant_i<R, T>;

//! @brief The randomised sequence of rounds for every node
using round_s = sequence::periodic<
    distribution::interval_n<times_t, 0, 1, 10>,
    distribution::weibull_i<times_t, tavg, tvar>
>;

//! @brief The distribution of initial node positions (to be overwritten by initial object positions).
using rectangle_d = distribution::rect<n<0>, n<0>, n<0>, i<sidex>, i<sidey>, n<0>>;

//! @brief Dictates that messages are thrown away after N second. Remember to change value according to schedule_type.
using retain_type = retain<metric::retain<fcpp::coordination::retain>>;

namespace data {
    //! @brief A representation of goal properties using tagged_tuple
    using goal_tuple_type = common::tagged_tuple_t<
        goal_action, string, 
        goal_code, string, 
        goal_pos_start_x, float, 
        goal_pos_start_y, float, 
        goal_orient_start_w, float,
        goal_pos_end_x, float, 
        goal_pos_end_y, float, 
        goal_orient_end_w, float,
        goal_source, string, 
        goal_priority, int, 
        goal_subcode, string,
        goal_step, int
    >; 

    //! @brief A representation of goal properties using tagged_tuple
    using external_status_tuple_type = common::tagged_tuple_t<
        node_ext_goal_status, feedback::GoalStatus, 
        node_ext_goal_update_time, system_clock::time_point
    >; 

    //! @brief A representation of rank data used in ap_engine
    // [0] -> rank of previous leader
    // [1] -> node uid of previous leader
    // [2] -> counter of how many rounds the leader is
    // [3] -> "lazy" leaders detection info
    //      [0] -> node uid of other leader detected
    //      [1] -> counter of how many rounds the detection info is stable
    using rank_data_type = tuple<real_t, device_t, int, tuple<device_t, int>>;
}

//! @brief The general simulation options.
DECLARE_OPTIONS(list,
    parallel<false>,     // no multithreading on node rounds
    synchronised<false>, // optimise for asynchronous networks
    program<coordination::main>,   // program to be run (refers to MAIN in process_management.hpp)
    exports<coordination::main_t>, // export type list (types used in messages)
    retain_type, // retain time for messages
    round_schedule<round_s>, // the sequence generator for round events on nodes
    log_schedule<sequence::periodic_n<1, 0, 1>>, // the sequence generator for log events on the network
    spawn_schedule<sequence::multiple<i<devices, size_t>, n<0>>>, // the sequence generator of node creation events on the network
    // the basic contents of the node storage
    tuple_store<
        seed,                               uint_fast32_t,
        speed,                              double,
        devices,                            size_t,
        sidex,                              real_t,
        sidey,                              real_t,
        node_color,                         color,
        left_color,                         color,
        right_color,                        color,
        node_size,                          double,
        node_shape,                         shape,
        node_label_size,                    double,
        node_label_text,                    string,
        node_label_color,                   color,
        node_shadow_shape,                  shape,
        node_shadow_size,                   double,
        node_shadow_color,                  color,
        node_ext_goal_status,               feedback::GoalStatus,
        node_ext_goal_current_step,         int,
        node_ext_docking_status,            feedback::DockStatus,
        node_ext_system_status,             feedback::SystemStatus,
        node_ext_goal_update_info,          std::unordered_map<std::string, fcpp::option::data::external_status_tuple_type>,
        node_ext_goal,                      string,
        node_ext_name,                      string,
        node_process_status,                ProcessingStatus,
        node_process_goal,                  string,
        node_process_computing_goals,       std::unordered_map<std::string, fcpp::option::data::goal_tuple_type>,
        node_battery_charge,                double,
        node_offset_pos_x,                  real_t,
        node_offset_pos_y,                  real_t,
        tavg,                               real_t,
        tvar,                               real_t
    >,
    // data initialisation
    init<
        x,                                  rectangle_d,
        seed,                               functor::cast<distribution::interval_n<double, 0, seed_max>, uint_fast32_t>,
        speed,                              functor::div<i<speed>, n<0>>,
        devices,                            i<devices>,
        tavg,                               i<tavg>, // seconds of mean period
        tvar,                               i<tvar>,  // seconds of variance period
        node_offset_pos_x,                  i<node_offset_pos_x>, 
        node_offset_pos_y,                  i<node_offset_pos_y>
    >,
    dimension<fcpp::coordination::dim>, // dimensionality of the space
    connector<connect::fixed<fcpp::coordination::comm, 1, fcpp::coordination::dim>>, // connection allowed within a fixed comm range
    shape_tag<node_shape>, // the shape of a node is read from this tag in the store
    size_tag<node_size>,   // the size of a node is read from this tag in the store
    color_tag<node_color, left_color, right_color>, // colors of a node are read from these
    label_size_tag<node_label_size>, // the size of the node label is read from this tag in the store
    label_text_tag<node_label_text>, // the text of the node label is read from this tag in the store
    label_color_tag<node_label_color>, // color of the node label is read from these
    shadow_shape_tag<node_shadow_shape>, // the shape of a shadow is read from this tag in the store
    shadow_size_tag<node_shadow_size>, // the size of the shadow is read from this tag in the store
    shadow_color_tag<node_shadow_color> // color of the node shape is read from these
);


}


}

#endif // NODES_AP_ENGINE_SETUP_H_
