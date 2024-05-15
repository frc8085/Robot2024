package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetTrapScore extends SequentialCommandGroup {
        public ResetTrapScore(
                        FeederSubsystem m_feeder,
                        ShooterSubsystem m_shooter) {
                addCommands(
                                new ParallelCommandGroup(
                                                new InstantCommand(m_feeder::runResetTrap),
                                                new InstantCommand(m_shooter::runResetTrap)),
                                new WaitUntilCommand(m_feeder::isNoteDetected),
                                new WaitUntilCommand(m_feeder::isNoteNotDetected),
                                new ParallelCommandGroup(
                                                new InstantCommand(m_feeder::stop),
                                                new InstantCommand(m_shooter::stop)),
                                new InstantCommand(m_shooter::runTrap));
        }
}