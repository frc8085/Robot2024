package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PickUpNoteCompleted extends SequentialCommandGroup {
        public PickUpNoteCompleted(
                        IntakeSubsystem m_intake, FeederSubsystem m_feeder, Blinkin m_blinkin) {
                addCommands(
                                new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                                new InstantCommand(m_intake::stop),
                                                                new InstantCommand(m_feeder::stop)),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(FeederConstants.kLoadWaitTime),
                                                                new InstantCommand(m_feeder::runBackwards),
                                                                new WaitUntilCommand(
                                                                                () -> m_feeder.isNoteNotDetected()),
                                                                new InstantCommand(m_feeder::stop),
                                                                new InstantCommand(m_blinkin::withNote))));
        }
}