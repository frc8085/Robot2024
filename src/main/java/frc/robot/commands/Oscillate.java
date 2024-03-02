package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Oscillate extends SequentialCommandGroup {
    public Oscillate(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            FeederSubsystem m_feeder,
            Blinkin m_blinkin) {
        addCommands(
                new WaitCommand(.1),
                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.TRAP_SCORE),
                new WaitCommand(.1),
                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.TRAP_FINAL));
    }

}
