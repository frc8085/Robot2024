package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PickUpNoteCompleted extends SequentialCommandGroup {
        public PickUpNoteCompleted(
                        IntakeSubsystem m_intake,
                        FeederSubsystem m_feeder,
                        Blinkin m_blinkin) {
                addCommands(
                                // Set boolean that the Note has been picked up
                                new InstantCommand(m_feeder::notePickedUp),
                                new ParallelCommandGroup(
                                                new InstantCommand(m_intake::stop),
                                                new InstantCommand(m_feeder::stop)),
                                new WaitCommand(FeederConstants.kLoadWaitTime),
                                // Check if Note is touching shooter wheels and run backwards if needed
                                new ConditionalCommand(new NoteCorrection(m_feeder), new InstantCommand(),
                                                m_feeder::needNoteCorrection),
                                // Change the LEDs to show note is in robot
                                new InstantCommand(m_blinkin::withNote));
        }
}