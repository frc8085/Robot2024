package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LimelightIntakeSubsystem;

public class TargetNoteTwice extends SequentialCommandGroup {
        public TargetNoteTwice(
                        LimelightIntakeSubsystem m_limelightIntake,
                        DriveSubsystem m_drive) {
                addCommands(
                                new ConditionalCommand(
                                                new AutoTargetNote(m_limelightIntake, m_drive),
                                                new InstantCommand(),
                                                m_limelightIntake::hasTarget),
                                new WaitCommand(0.1),
                                new ConditionalCommand(
                                                new AutoTargetNote(m_limelightIntake, m_drive),
                                                new InstantCommand(),
                                                m_limelightIntake::hasTarget));
        }
}
