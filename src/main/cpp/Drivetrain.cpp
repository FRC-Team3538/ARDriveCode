// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  const double leftOutput = m_leftPIDController.Calculate(
      m_leftEncoder.GetRate(), speeds.left.to<double>());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightEncoder.GetRate(), speeds.right.to<double>());

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(double forward, double rotation) {
    m_DriveL1.Set(forward - rotation);
    m_DriveL2.Set(forward - rotation);
    m_DriveR1.Set(forward + rotation);
    m_DriveR2.Set(forward + rotation);
}

void Drivetrain::Climb(double front, double backward) {
  m_Climb.Set(front - backward);
}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
}
