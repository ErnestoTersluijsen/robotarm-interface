//
// Created by ernesto on 29-2-24.
//

#ifndef ROBOTARM_HLD_ROBOTARMHLD_HPP
#define ROBOTARM_HLD_ROBOTARMHLD_HPP

#include "robotarm_lld/RobotarmLLD.hpp"

class RobotarmHLD
{
  public:
	RobotarmHLD(const std::string& port_name);

	void move_servos(std::vector<uint16_t> servo_ids, std::vector<int16_t> joint_angles, uint16_t speed);

  private:
	RobotarmLLD lld;
};

#endif // ROBOTARM_HLD_ROBOTARMHLD_HPP
