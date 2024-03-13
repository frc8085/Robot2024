package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightShoot extends SequentialCommandGroup {
        public LimelightShoot(
                        FeederSubsystem m_feeder,
                        ArmSubsystem m_arm,
                        ShooterSubsystem m_shooter,
                        Blinkin m_blinkin,
                        LimelightSubsystem m_limelight,
                        DriveSubsystem m_drive) {
                addCommands(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.PODIUM),
                                new ParallelCommandGroup(
                                                new TargetTwice(m_limelight, m_drive),
                                                new TargetSPTwice(m_limelight, m_arm)),
                                new ShootPodium(m_feeder, m_arm, m_shooter, m_blinkin));
        }
}
