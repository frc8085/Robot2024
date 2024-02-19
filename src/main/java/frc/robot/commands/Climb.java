package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Climb extends SequentialCommandGroup {
    public Climb(
            FeederSubsystem m_feeder,
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Position position) {
        addCommands(
                new MoveToPosition(m_arm, m_shooter, Position.TRAP_SCORE),
                new InstantCommand(m_shooter::runTrap),
                new InstantCommand(m_feeder::run),
                new WaitCommand(1),
                new InstantCommand(m_feeder::stop),
                new InstantCommand(m_shooter::stop));

    }
}
