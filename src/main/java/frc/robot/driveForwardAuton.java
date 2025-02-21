package frc.robot;

import javax.naming.directory.InvalidSearchFilterException;

import com.fasterxml.jackson.annotation.JsonAlias;
import com.fasterxml.jackson.databind.util.JSONPObject;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.testrobotautonmovement.RobotMovement.Motor;


public class driveForwardAuton {
    SwerveModule rightFront;
    SwerveModule rightBack;
    SwerveModule leftFront;
    SwerveModule leftBack;
    SparkMax liftMotor;
    String state = "start";
    SwerveDriveKinematics drive1;
    AHRS gyro1;

    public driveForwardAuton(SwerveModule[] List, SparkMax Lift, SwerveDriveKinematics drive, AHRS gyro) {
        rightFront = List[0];
        rightBack = List[1];
        leftFront = List[2];
        leftBack = List[3];
        liftMotor = Lift;
        drive1 = drive;
        gyro1 = gyro;
    }

    void RunS2R(int time) {
        switch (state) {
            // first number in if statement is time in seconds
            case "start":
            MotorSet(0.2,0.0,0.0);
            if (time > 100) {
                state = "stop";
            }
            break;

            case "stop":
            MotorSet(0.0,0.0,0.0);
                state = "Tag1";
            break;

            case "Tag1":
            /*NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getValue();
            NetworkTableEntry jsonEntry = table.getEntry("json");

            String jsonString = jsonEntry.getString("{}");
            JSONObject json = new JSONObject(jsonString);
            JSONArray targets = json.getJSONArray("targets");

            int desiredTagId = 13;
            for (int i = 0; i < targets.length (); i++){
                JSONObject target = targets.getJSONOnject(i);
                int tagId = target.getInt("tid");
                if (tagId == desiredTagId) {
                    double tx = target.getdouble("tx");
                    double ty = target.getdouble("ty");
                }
            }*/



        }
    }



    void MotorSet(double Y, double X, double rotation){
    
        ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(Y, X, rotation, gyro1.getRotation2d());
        SwerveModuleState[] moduleStates = drive1.toSwerveModuleStates(control);

  moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
  moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
  moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
  moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));
  
  rightFront.turny(moduleStates[0].angle.getDegrees());
  leftFront.turny(moduleStates[1].angle.getDegrees());
  rightBack.turny(moduleStates[2].angle.getDegrees());
  leftBack.turny(moduleStates[3].angle.getDegrees());


    rightFront.movey(moduleStates[0].speedMetersPerSecond);
    leftFront.movey(moduleStates[1].speedMetersPerSecond);
    rightBack.movey(moduleStates[2].speedMetersPerSecond);
    leftBack.movey(moduleStates[3].speedMetersPerSecond);


    }
}