package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Climb extends SequentialCommandGroup {
  public Climb(
      FeederSubsystem m_feeder,
      ArmSubsystem m_arm,
      ShooterSubsystem m_shooter,
      ClimberSubsystem m_climber,
      Blinkin m_blinkin) {
    addCommands(
        // Set Position to Trap Approach
        new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.TRAP_APPROACH),

        // Move robot centered on chain against wall using limelight

        // Start Climbing until it can move to the next position
        new InstantCommand(m_climber::raise),

        // when it gets to the correct height, change to trap score position and turn on
        // shooter, but keep climbing up
        new WaitUntilCommand(m_climber::atMoveToTrapScore),
        new MoveToPosition(m_arm, m_shooter, m_feeder, m_blinkin, Position.TRAP_SCORE),
        new InstantCommand(m_shooter::runTrap),

        // when it gets to the trap height, stop climbing (ideally we'd hold with a PID)
        new WaitUntilCommand(m_climber::atTrapScore),
        new InstantCommand(m_climber::stop),

        // shoot note
        new ShootTrap(m_feeder, m_arm, m_shooter, m_blinkin));
  }
}
