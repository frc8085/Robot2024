package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ArmConstants.Position;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightShoot extends SequentialCommandGroup {
        public LimelightShoot(
                        FeederSubsystem m_feeder,
                        ArmSubsystem m_arm,
                        ShooterSubsystem m_shooter,
                        Blinkin m_blinkin,
                        LimelightSubsystem m_limelight,
                        DriveSubsystem m_drive) {
                addCommands(
                                new ParallelCommandGroup(
                                                new TargetTwice(m_limelight, m_drive),
                                                new ConditionalCommand(new TargetSPTwice(m_limelight, m_arm),
                                                                new InstantCommand(), m_arm::atPodiumPosition)),
                                new ShootNew(m_feeder, m_arm, m_shooter, m_blinkin, Position.HOME));
        }
}
