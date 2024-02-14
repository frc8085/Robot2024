//template for what most targeting movement scripts have to do to line up
//as far as I cant tell there is a way to get all the April tags in veiw however I can't find it so this may need some tweaking later
package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;

public class aligntemplate {
    public void main() {
        double ID = LimelightSubsystem.getAprilTagID();
        if (ID == DriveConstants.templateID) {
            double[] targpos = frc.robot.Constants.DriveConstants.templatePosition;
            double[] robotpos = frc.robot.subsystems.LimelightSubsystem.getRobotLocation();
            // I have to make some generous guesses
            double offset[] = { (int) (targpos[0] - robotpos[0]), (int) (targpos[1] - robotpos[4]) };
            // Unit convestions ucx=x after unit conversion ucz=z after unit conversion
            double ucx = 0;
            double ucz = 0;
            // TODO: make not error
            // DriveSubsystem.drive(ucx, ucz, DriveConstants.targRot, true, false);
        }
        ;
        // Vscode is fighting with me :(
    };
}