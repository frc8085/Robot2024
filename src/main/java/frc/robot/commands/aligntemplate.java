//template for what most targeting movement scripts have to do to line up
//as far as I cant tell there is a way to get all the April tags in veiw however I can't find it so this may need some tweaking later
package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class aligntemplate {
    public void main() {
        DriveSubsystem drive = new DriveSubsystem();
        double ID = LimelightSubsystem.getAprilTagID();
        while (LimelightSubsystem.getdegRotationToTarget() > .5) {
            // we need a unit conversion
            double turnamt = LimelightSubsystem.getdegRotationToTarget() * DriveConstants.degtospeed;
            drive.turn(turnamt);
        }
        if (ID == DriveConstants.templateID) {
            double[] targpos = frc.robot.Constants.DriveConstants.templatePosition;
            double[] robotpos = frc.robot.subsystems.LimelightSubsystem.getDistanceToTarget();
            // I have to make some generous guesses
            double offset[] = { (int) (targpos[0] - robotpos[0]), (int) (targpos[1] - robotpos[4]) };
            // Unit convestions ucx=x after unit conversion ucz=z after unit conversion
            double ucx = offset[0] * LimelightConstants.unittometer;
            double ucz = offset[1] * LimelightConstants.unittometer;
            drive.drive(1, ucx, ucz, 0, true, false);
        }
        ;
        // Vscode is fighting with me :(
    };
}