package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;


public class MoveToTravel extends SequentialCommandGroup {
    public MoveToTravel(
            ArmSubsystem m_arm) {
        addCommands(
            new InstantCommand(() -> m_arm.keepArmPosition(ArmConstants.travelArmPosition)));
    }

}
