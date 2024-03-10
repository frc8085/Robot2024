// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoTargetSP extends SequentialCommandGroup {

  public AutoTargetSP(LimelightSubsystem m_limelight, ArmSubsystem m_arm) {
    addCommands(
        new InstantCommand(() -> m_arm.setShooterPivotPosition(m_limelight.getShooterPivotSetpointFromArea())));
  }
}
