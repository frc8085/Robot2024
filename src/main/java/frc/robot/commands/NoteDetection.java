package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class NoteDetection extends SequentialCommandGroup {

    public double kTurnSpeed = 1;
    public double kForwardSpeed = 1;

    public NoteDetection(LimelightSubsystem m_LimelightSubsystem, DriveSubsystem m_DriveSubsystem,
            IntakeSubsystem m_IntakeSubsystem, SmartDashboard m_SmartDashboard, FeederSubsystem m_FeederSubsystem) {

        m_LimelightSubsystem.setPipeline(3);

        if (m_LimelightSubsystem.hasTarget()) {

            double xOffset = m_LimelightSubsystem.getXfromRobotPerspective();

            if (xOffset > 0) {
                while (-0.5 < xOffset || xOffset < 0.5) {
                    m_DriveSubsystem.turn(kTurnSpeed);
                    xOffset = m_LimelightSubsystem.getXfromRobotPerspective();
                }

            } else if (xOffset < 0) {
                while (-0.5 < xOffset || xOffset < 0.5) {
                    m_DriveSubsystem.turn(-kTurnSpeed);
                    xOffset = m_LimelightSubsystem.getXfromRobotPerspective();
                    new WaitCommand(0.001);
                }
            }

            m_IntakeSubsystem.run();

            m_DriveSubsystem.drive(kForwardSpeed, 1, 0, 0, false, false);

            while (true) {
                if (m_FeederSubsystem.isNoteDetected() == true) {
                    m_DriveSubsystem.stop();
                    m_IntakeSubsystem.stop();
                    break;
                }

            }

            // Not written yet:
            // Detects note in robot and disables drive motors
        } else {
            System.out.println("No Target");
        }
    }
}
