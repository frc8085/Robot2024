// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class AutoShooterTarget extends PIDCommand {
    private ArmSubsystem m_arm;
    private LimelightSubsystem m_limelight;
    private double m_degree;

    static double kP = 0.02;
    static double kI = 0;
    static double kD = 0.000;

    /**
     * Create a new TurnToDegreeGyro command.
     *
     * @param angle The angle to rotate the shooter pivot
     */
    public AutoShooterTarget(LimelightSubsystem limelight, ArmSubsystem arm, boolean relative) {
        super(new PIDController(kP, kI, kD),
                // Close loop on heading
                arm::getShooterPivotPosition,
                // Set reference to target
                limelight::getY,
                // Pipe output to turn robot
                output -> arm.adjustShooter(output));

        m_arm = arm;
        m_limelight = limelight;
        m_degree = m_limelight.getY();

        addRequirements(m_limelight, m_arm);

        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the setpoint before it is considered as having reached the
        // reference
        getController()
                .setTolerance(2, 2);

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_arm.shooterPivotStop();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

        super.initialize();
        m_degree = m_limelight.getY();
        SmartDashboard.putNumber("Desired SP adjustment", m_degree);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
