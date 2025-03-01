package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class driveTrain {
    boolean zeroMode = false;
    boolean oldDriveBase = false;
    
    public SwerveModule leftFront;
    public SwerveModule rightFront;
    public SwerveModule rightBack;
    public SwerveModule leftBack;

    SwerveDriveKinematics kinematics;

    SwerveDriveOdometry odometry;

    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    Dashboard dashboard;

    SparkMax liftMotor = null;
    double powerMulti = 0.6;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, tol; //the PID loop doubles 

    driveTrain(Dashboard dash) {
        gyro.reset();
        dashboard = dash;
        
        if (!oldDriveBase) {
            // competition base CAN IDs
            leftFront = new SwerveModule(0,66.3065, 14,6, zeroMode,oldDriveBase);
            rightFront = new SwerveModule(2,-134.8564, 33,4, zeroMode,oldDriveBase);
            rightBack = new SwerveModule(3,64.7032, 10,11, zeroMode,oldDriveBase);
            leftBack = new SwerveModule(1,85.9213, 19,16, zeroMode,oldDriveBase);


         //   liftMotor = new SparkMax(12, MotorType.kBrushed);
        } 
        else {
            // old drive base CAN IDs
            leftFront = new SwerveModule(0,348.0-90, 12,17, zeroMode,oldDriveBase);
            rightFront = new SwerveModule(1,70.1-270, 20,2, zeroMode,oldDriveBase);
            rightBack = new SwerveModule(2,2.31-180, 14,32, zeroMode,oldDriveBase);
            leftBack = new SwerveModule(3,69.3-180, 29,15, zeroMode,oldDriveBase);
        }

        Translation2d frontRight = new Translation2d(0.33655, -0.33655); 
        Translation2d frontLeft = new Translation2d(0.33655, 0.33655); 
        Translation2d backRight = new Translation2d(-0.33655, -0.33655); 
        Translation2d backLeft = new Translation2d(-0.33655, 0.33655);     
  
        kinematics = new SwerveDriveKinematics(frontRight, frontLeft, backRight, backLeft);

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), 
            new SwerveModulePosition[]{
                rightFront.getPosition(),
                leftFront.getPosition(),
                rightBack.getPosition(),
                leftBack.getPosition(),    
            });
    }

    void drive(double xSpeed, double ySpeed, double rot, boolean highSpeed) {
        ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);
    
        moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
        moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
        moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
        moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));
    
        rightFront.turny(moduleStates[0].angle.getDegrees());
        leftFront.turny(moduleStates[1].angle.getDegrees());
        rightBack.turny(moduleStates[2].angle.getDegrees());
        leftBack.turny(moduleStates[3].angle.getDegrees());
      
        if (highSpeed)  {
          rightFront.movey(moduleStates[0].speedMetersPerSecond*0.87);
          leftFront.movey(moduleStates[1].speedMetersPerSecond*0.87);
          rightBack.movey(moduleStates[2].speedMetersPerSecond*0.87);
          leftBack.movey(moduleStates[3].speedMetersPerSecond*0.87);
      
        }else {
          rightFront.movey(moduleStates[0].speedMetersPerSecond/2);
          leftFront.movey(moduleStates[1].speedMetersPerSecond/2);
          rightBack.movey(moduleStates[2].speedMetersPerSecond/2);
          leftBack.movey(moduleStates[3].speedMetersPerSecond/2); 
        }          
        dashboard.updateDashboardSwerveModules(leftFront,rightFront,leftBack,rightBack);
    }

    void resetGyro()
    {
        gyro.reset();
    }

    public void updateOdometry() {
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                rightFront.getPosition(),
                leftFront.getPosition(),
                rightBack.getPosition(),
                leftBack.getPosition()
            });
      }
      RawFiducial GetAprilTagTelemotry(int aprilTag) {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
            for (RawFiducial fiducial : fiducials) {
                    int id = fiducial.id;
                    double txnc = fiducial.txnc;
                    double tync = fiducial.tync;
                    double ta = fiducial.ta;
                    double distToCamera = fiducial.distToCamera;
                    double distToRobot = fiducial.distToRobot;
                    double ambiguity = fiducial.ambiguity; 

                    if(id == aprilTag) {
                        return fiducial;
                    }
                    if(txnc == aprilTag){
                        return fiducial;
                    }
                    if(tync == aprilTag){
                        return fiducial;
                    }
                    if(ta == aprilTag){
                        return fiducial;
                    }
                System.out.println("Tag: " + id);
            }
            return null;
      }
}