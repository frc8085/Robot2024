package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EnableShooterAmp extends SequentialCommandGroup {
        public EnableShooterAmp(
                        FeederSubsystem m_feeder,
                        ShooterSubsystem m_shooter) {
                addCommands(
                                new NoteCorrection(m_feeder),
                                new InstantCommand(m_shooter::runAmp));
        }
}
