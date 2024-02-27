package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootNew extends SequentialCommandGroup {
    public ShootNew(
            FeederSubsystem m_feeder,
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Blinkin m_blinkin,
            Position position) {
        addCommands(
                new WaitUntilCommand(m_shooter::readyToShoot),
                new InstantCommand(m_feeder::run),
                // watch for isNoteDetected to be true and then turn off the feeder after X
                // seconds
                new WaitUntilCommand(m_feeder::isNoteDetected),
                new WaitUntilCommand(m_feeder::isNoteNotDetected),
                new WaitCommand(.1),
                new InstantCommand(m_feeder::stop),
                new InstantCommand(m_shooter::stop),
                new InstantCommand(m_feeder::noteShot),
                new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, position),
                new InstantCommand(m_blinkin::driving));
    }
}
