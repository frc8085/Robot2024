package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends SequentialCommandGroup{
    public IntakeStop(
        IntakeSubsystem m_intake, FeederSubsystem m_feeder
    ){
    addCommands(
        new InstantCommand(m_intake::stop),
        new InstantCommand(m_feeder::stop));
    }
}
