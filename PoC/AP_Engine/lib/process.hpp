// Copyright © 2023 Gianluca Torta, Daniele Bortoluzzi. All Rights Reserved.

/**
 * @file process.hpp
 * @brief macro utils for process management.
 */

#ifndef NODES_PROCESS_H_
#define NODES_PROCESS_H_

// MACRO
#define PROCESS(NewGoalsList, code) \
    spawn_result_type r =  coordination::spawn(CALL, [&](goal_tuple_type const& g){ \
        status s = status::internal_output; \
        code \
        termination_logic(CALL, s, g); \
        return make_tuple(node.current_time(), s); \
    }, NewGoalsList); \
    manage_termination(CALL, r); \

#endif
