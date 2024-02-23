package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSimple extends SequentialCommandGroup {
    public AutoSimple(
            FeederSubsystem m_feeder,
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Blinkin m_blinkin,
            Position position) {
        addCommands(
                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.SUBWOOFER),
                new Shoot(m_feeder, m_arm, m_shooter, m_blinkin, Position.HOME));

    }

}
