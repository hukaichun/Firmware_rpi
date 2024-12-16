# Concept
The **hrl controller** is a controller module for a quadrotor.

The controller observes the vehicle's attitude, position, and outputs thrust directly. The thrust is computed by a neural network trained using reinforcement learning.

# Navigation
This PX4 module has two responsibilities: 1) CLI interface, and 2) the control function.
### CLI interface
hrl_control.cpp 

- hrl_control_main

### Control loop
hierarchical_controller.cpp

- HierarchicalController::main_loop


# Objects  
### class HierarchicalController
The HierarchicalController consists of three ports:
- HierarchicalController::_policy: the control policy 
- HierarchicalController::_log: the logging module
- HierarchicalController::_uORB: the communication module within the vehicle


### _uORB
_uORB is responsible for vehicle information communication:

1. Retrieve the vehicle status.
2. Submit the control signal.

### _policy
_policy is the control policy built upon a neural network, expressed as:
```
thrust = _policy(vehicle_status)
```

### _log
_log is a socket server that transmits vehicle status to ground stations.

