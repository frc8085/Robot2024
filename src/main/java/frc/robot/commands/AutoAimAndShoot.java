package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.Position;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAimAndShoot extends SequentialCommandGroup {
    public AutoAimAndShoot(
            ArmSubsystem m_arm,
            ShooterSubsystem m_shooter,
            Position position) {
        addCommands(
                new InstantCommand(() -> System.out.println("**START Move to " + position.label + " Position**")),
                new InstantCommand(() -> m_arm.moveToPosition(position)),
                new InstantCommand(m_shooter::run));
    }

}
