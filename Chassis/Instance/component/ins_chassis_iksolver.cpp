/**
 *******************************************************************************
 * @file      : ins_chassis_iksolver.cpp
 * @brief     : 底盘逆解器单例，几何参数来自已补全考核说明 6.2
 *******************************************************************************
 */
#include "ins_chassis_iksolver.hpp"

#include "chassis_ik_solver.hpp"

hello_world::chassis_ik_solver::ChassisIkSolver *GetChassisIkSolver() {
  static hello_world::chassis_ik_solver::ChassisIkSolver solver;
  return &solver;
}
