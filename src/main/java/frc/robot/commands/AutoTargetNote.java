// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LimelightIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to
 * run a simple PID loop that is only enabled while this command is running. The
 * input is the
 * averaged values of the left and right encoders.
 */
public class AutoTargetNote extends PIDCommand {
    private final DriveSubsystem m_drive;
    private LimelightIntakeSubsystem m_limelightIntake;
    private double m_degree;

    static double kP = 0.008;
    static double kI = 0;
    // .0008
    static double kD = 0;

    /**
     * Create a new TurnToDegreeGyro command.
     *
     * @param distance The distance to drive (inches)
     */
    public AutoTargetNote(LimelightIntakeSubsystem limelightIntake, DriveSubsystem drive) {
        super(new PIDController(kP, kI, kD),
                // Close loop on heading
                drive::getHeadingWrappedDegrees,
                // Set reference to target
                limelightIntake::getXfromRobotPerspective,
                // Pipe output to turn robot
                output -> drive.turn(output));

        // Require the drive
        m_drive = drive;
        m_limelightIntake = limelightIntake;
        m_degree = m_limelightIntake.getXfromRobotPerspective();

        addRequirements(m_limelightIntake, m_drive);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);

        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the setpoint before it is considered as having reached the
        // reference
        getController()
                .setTolerance(5, AutoConstants.kTurnRateToleranceDegPerS);

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        m_drive.stop();
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

        super.initialize();
        m_degree = m_limelightIntake.getXfromRobotPerspective();
        SmartDashboard.putNumber("Desired turning deg", m_degree);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
