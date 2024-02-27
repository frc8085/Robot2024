package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class NoteCorrection extends SequentialCommandGroup {
        public NoteCorrection(
                        FeederSubsystem m_feeder) {
                addCommands(
                                new InstantCommand(m_feeder::stop),
                                new WaitCommand(FeederConstants.kLoadWaitTime),
                                new InstantCommand(m_feeder::runBackwards),
                                new WaitUntilCommand(
                                                () -> m_feeder.isNoteNotDetected()),
                                new InstantCommand(m_feeder::stop));
        }
}
