#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ros_manager_init(void);
//bool ros_manager_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // ROS_MANAGER_H