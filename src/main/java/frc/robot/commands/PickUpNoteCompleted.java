package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PickUpNoteCompleted extends SequentialCommandGroup{
    public PickUpNoteCompleted(
        IntakeSubsystem m_intake, FeederSubsystem m_feeder
    ){
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(m_intake::stop),
                new InstantCommand(m_feeder::stop)),
            new SequentialCommandGroup(
                new InstantCommand(m_feeder::runBackwards),
                new WaitUntilCommand(() -> m_feeder.isNoteNotDetected()),
                new InstantCommand(m_feeder::stop))));
        }
}