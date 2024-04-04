package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTarget extends SequentialCommandGroup {
        public LimelightTarget(
                        ArmSubsystem m_arm,
                        LimelightSubsystem m_limelight,
                        DriveSubsystem m_drive) {
                addCommands(
                                new ParallelCommandGroup(
                                                new TargetTwice(m_limelight, m_drive),
                                                new ConditionalCommand(new TargetSPTwice(m_limelight, m_arm),
                                                                new InstantCommand(), m_arm::atPodiumPosition)));
        }
}
