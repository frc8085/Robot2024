package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* Command to run at the end of a path movement to check if the note has been picked up and if not, continue on the auto */
public class NoteCheckAuto extends SequentialCommandGroup {
        public NoteCheckAuto(
                        IntakeSubsystem m_intake,
                        FeederSubsystem m_feeder) {
                addCommands(
                                new WriteToLog("started note check auto"),
                                // Check if Note has been picked up already, and if it hasn't, wait one
                                // sec then end
                                new ConditionalCommand(new InstantCommand(),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(.5),
                                                                new ParallelCommandGroup(
                                                                                new InstantCommand(m_intake::stop),
                                                                                new InstantCommand(m_feeder::stop))),
                                                m_feeder::noteInRobot));

        }
}
