package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.Position;

public class MoveToPodium extends SequentialCommandGroup {
    public MoveToPodium(
            ArmSubsystem m_arm,
            Position position) {
        addCommands(
            new InstantCommand(() -> m_arm.keepArmPosition(ArmConstants.podiumArmPosition)));
    }

}
