package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class TargetTwice extends SequentialCommandGroup {
    public TargetTwice(
            LimelightSubsystem m_limelight,
            DriveSubsystem m_drive,
            ArmSubsystem m_arm) {
        addCommands(
                new ConditionalCommand(
                        new AutoTarget(m_limelight, m_drive, true),
                        new InstantCommand(),
                        m_limelight::hasTarget),
                new WaitCommand(0.1),
                new ConditionalCommand(
                        new AutoTarget(m_limelight, m_drive, true),
                        new InstantCommand(),
                        m_limelight::hasTarget));
        /*
         * new WaitCommand(0.1),
         * new ConditionalCommand(
         * new AutoYAim(m_limelight, m_arm),
         * new InstantCommand(),
         * m_limelight::hasTarget),
         * new WaitCommand(0.1),
         * new ConditionalCommand(
         * new AutoYAim(m_limelight, m_arm),
         * new InstantCommand(),
         * m_limelight::hasTarget));
         */
    }
}
