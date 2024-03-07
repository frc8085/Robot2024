package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectNote extends SequentialCommandGroup {
        public EjectNote(
                        IntakeSubsystem m_intake,
                        FeederSubsystem m_feeder,
                        ArmSubsystem m_arm,
                        ShooterSubsystem m_shooter,
                        Blinkin m_blinkin) {
                addCommands(
                                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.EJECT_NOTE),
                                new ParallelCommandGroup(
                                                new InstantCommand(m_intake::eject),
                                                new InstantCommand(m_feeder::eject)));
        }
}
