// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "Drivetrain.h"
#include "PS4Controller.h"

using namespace frc;

class Robot : public frc::TimedRobot
{
public:
  void AutonomousPeriodic() override
  {
    TeleopPeriodic();
    m_drive.UpdateOdometry();
  }

  void TeleopPeriodic() override
  {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    auto forward = m_controller.GetY(frc::GenericHID::kLeftHand);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we p\ull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rotation = m_controller.GetX(frc::GenericHID::kRightHand);
  forward = -Deadbandforward(forward, 0.05);
  rotation = -Deadbandforward(rotation, 0.05);
  m_drive.Drive(forward, rotation);
   
   
    auto front = m_controller.GetTriggerAxis(frc::GenericHID::kLeftHand);
    front = Deadbandforward(front, 0.05);
    auto backward = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);
    backward = Deadbandforward(backward, 0.05);
    m_drive.Climb(front, backward);
  }

  double Deadbandforward(double forward, double deadband)
  {
    if ((std::abs(forward)) < deadband)
    {
      return 0.0;
    }
    else if (forward > 0.95)
    {
      return 1.0;
    }
    else if (forward < -0.95)
    {
      return -1.0;
    }
    else
    {
      return forward;
    }
  }

  double Deadbandrotation(double rotation, double deadband)
  {
    if ((std::abs(rotation)) < deadband)
    {
      return 0.0;
    }
    else if (rotation > 0.95)
    {
      return 1.0;
    }
    else if (rotation < -0.95)
    {
      return -1.0;
    }
    else
    {
      return rotation;
    }
  }

private:
  PS4Controller m_controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive;
  
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
