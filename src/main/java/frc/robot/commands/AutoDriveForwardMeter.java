// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoDriveForwardMeter extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_drive;
    private double m_meters = 0;

    public AutoDriveForwardMeter(DriveSubsystem drive, double meters) {
        m_drive = drive;
        m_meters = meters;
        addRequirements(m_drive);
    }

    // Reset the odomotry when the command is scheduled
    // Then run the drive comma
    public void initialize() {
        super.initialize();
        m_drive.resetOdometry(new Pose2d());
        m_drive.drive(
                .4,
                1,
                0,
                0,
                true,
                false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Stop driving when the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // End the command when we reach the desired pose in meters
    @Override
    public boolean isFinished() {
        double currentPose = m_drive.getPose().getX();
        // Stop when the current position reaches
        // the desired forward travel distance in meters
        return currentPose >= m_meters;
    }
}