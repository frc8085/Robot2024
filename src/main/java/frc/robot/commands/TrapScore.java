package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrapScore extends SequentialCommandGroup {
    public TrapScore(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            FeederSubsystem m_feeder,
            Blinkin m_blinkin) {
        new SequentialCommandGroup(
                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.TRAP_SCORE),
                new ConditionalCommand(new NoteCorrection(m_feeder, m_shooter)
                        .andThen(new InstantCommand(m_shooter::runTrap)),
                        new InstantCommand(m_shooter::runTrap),
                        m_feeder::needNoteCorrection));
    }
}
