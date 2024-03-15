package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class TargetTwice extends SequentialCommandGroup {
        public TargetTwice(
                        LimelightSubsystem m_limelight,
                        DriveSubsystem m_drive) {
                addCommands(
                                new InstantCommand(() -> Logger.recordOutput("Commands/LimelightTarget", false)),
                                new ConditionalCommand(
                                                new AutoTarget(m_limelight, m_drive),
                                                new InstantCommand(),
                                                m_limelight::hasTarget),
                                new WaitCommand(0.1),
                                new ConditionalCommand(
                                                new AutoTarget(m_limelight, m_drive),
                                                new InstantCommand(),
                                                m_limelight::hasTarget),
                                new InstantCommand(() -> Logger.recordOutput("Commands/LimelightTarget", true)));
        }
}
