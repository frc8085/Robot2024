package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PickUpNote extends SequentialCommandGroup {
    public PickUpNote(
            IntakeSubsystem m_intake,
            FeederSubsystem m_feeder,
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Blinkin m_blinkin) {
        addCommands(
                // Don't pick up until we're in travel
                new MoveToPosition(m_arm, m_shooter, m_blinkin, Position.HOME),
                new WaitUntilCommand(m_arm::atTravelPosition),
                // Pick up after we reach travel
                new ParallelCommandGroup(
                        new InstantCommand(m_blinkin::intakeOn),
                        new InstantCommand(m_intake::run),
                        new InstantCommand(m_feeder::run)));
    }
}
