package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootManualTrap extends SequentialCommandGroup {
    public ShootManualTrap(
            FeederSubsystem m_feeder,
            ShooterSubsystem m_shooter) {
        addCommands(
                new InstantCommand(m_feeder::runBackwards),
                new WaitUntilCommand(() -> m_feeder.isNoteNotDetected()),
                new InstantCommand(m_shooter::run));
    }
}
