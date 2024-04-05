// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file ap_engine.cpp
 * @brief Runs the AP engine for the NODES demo in simulation environment.
 */

#define RUN_SIMULATION

#include "lib/poc_config.hpp"
#include "lib/ap_engine.hpp"
#include "lib/ap_engine_setup.hpp"

#include "lib/poc_file_watcher.hpp"

using namespace fcpp;

int main() {
    // file watcher
    poc_file_watcher poc_file_watcher;
    poc_file_watcher.run_watch_files();
 
    int speed = 0;
    int robots = ROBOTS_COUNT;  
    std::string background_image = BG_IMAGE;

    // The network object type (interactive simulator with given options).
    using net_t = component::interactive_simulator<option::list>::net;
    // The initialisation values (simulation name, ...).
    auto init_v = common::make_tagged_tuple
    <
        option::name, 
        option::texture, 
        option::speed, 
        option::devices, 
        option::seed, 
        option::area_min, 
        option::area_max, 
        fcpp::coordination::tags::tavg, 
        fcpp::coordination::tags::tvar,
        fcpp::coordination::node_offset_pos_x, 
        fcpp::coordination::node_offset_pos_y
    >(
        "ROS2 Sample Project",
        background_image,
        0,
        robots + 1 /*KIOSK*/,
        1,
        fcpp::coordination::amin,
        fcpp::coordination::amax,
        ROUND_PERIOD,
        (real_t)1/100,  // var: 1/100s
        SIMULATOR_OFFSET_X,
        SIMULATOR_OFFSET_Y
    );
    // Construct the network object.
    net_t network{init_v}; 
    network.run();
 
    return 0;
}

