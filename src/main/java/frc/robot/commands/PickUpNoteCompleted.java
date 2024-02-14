package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PickUpNoteCompleted extends SequentialCommandGroup{
    public PickUpNoteCompleted(
        IntakeSubsystem m_intake, FeederSubsystem m_feeder
    ){
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(m_intake::stop),
            new InstantCommand(m_feeder::stop)));
    }
}
