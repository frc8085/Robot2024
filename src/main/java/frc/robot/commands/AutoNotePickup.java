package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class AutoNotePickup extends SequentialCommandGroup {

    public double kTurnSpeed = 0.3;
    public double kForwardSpeed = 0.18;

    public AutoNotePickup(LimelightSubsystem m_LimelightSubsystem, DriveSubsystem m_DriveSubsystem,
            IntakeSubsystem m_IntakeSubsystem, FeederSubsystem m_FeederSubsystem) {

        // m_LimelightSubsystem.setPipeline(3);

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
