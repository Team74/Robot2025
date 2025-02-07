package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDrive {

    boolean zeroMode = false;
    boolean oldDriveBase = false;
    SwerveDriveKinematics driveLocation;


    public SwerveDrive() {
        
        SwerveModule rightFront = new SwerveModule(1,48.6278,33,4,zeroMode,oldDriveBase);
        SwerveModule leftFront = new SwerveModule(0,-112.6435,14,6,zeroMode,oldDriveBase);
        SwerveModule rightBack = new SwerveModule(2,-105.9345,19,16,zeroMode,oldDriveBase);
        SwerveModule leftBack = new SwerveModule(3,-91.9409,10,11,zeroMode,oldDriveBase);

        Translation2d rightFrontLocation = new Translation2d(0.33655, -0.33655); 
        Translation2d leftFrontLocation = new Translation2d(0.33655, 0.33655); 
        Translation2d rightBackLocation = new Translation2d(-0.33655, -0.33655); 
        Translation2d leftBackLocation = new Translation2d(-0.33655, 0.33655); 
        driveLocation = new SwerveDriveKinematics(rightFrontLocation, leftFrontLocation, rightBackLocation, leftBackLocation);
    }



}
