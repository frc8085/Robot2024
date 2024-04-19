package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class WriteToLog extends SequentialCommandGroup {
    public WriteToLog(
            String info) {
        addCommands(new InstantCommand(() -> System.out.println(info)),
                new InstantCommand(() -> Logger.recordOutput("Commands/Log", info)));
    }
}
