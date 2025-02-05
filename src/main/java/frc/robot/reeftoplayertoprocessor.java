// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class reeftoplayertoprocessor extends TimedRobot {
  boolean zeroMode = false;
  XboxController controller = new XboxController(0);
  Dashboard dashboard = new Dashboard();

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  SwerveModule rightFront;// = new SwerveModule(1,48.6278,33,4,zeroMode);
  SwerveModule leftFront;// = new SwerveModule(0,-112.6435,14,6,zeroMode);
  SwerveModule rightBack;// = new SwerveModule(2,-105.9345,19,16,zeroMode);
  SwerveModule leftBack;// = new SwerveModule(3,-91.9409,10,11,zeroMode);

  Translation2d frontRight = new Translation2d(0.33655, -0.33655); 
  Translation2d frontLeft = new Translation2d(0.33655, 0.33655); 
  Translation2d backRight = new Translation2d(-0.33655, -0.33655); 
  Translation2d backLeft = new Translation2d(-0.33655, 0.33655); 
  private final Timer timerAuton = new Timer();
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontRight, frontLeft, backRight, backLeft);
  int time = 0;  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
     
  public reeftoplayertoprocessor(SwerveModule _rightFront, SwerveModule _leftFront, SwerveModule _rightBack , SwerveModule _leftBack) {
    rightFront = _rightFront;
    leftFront = _leftFront;
    rightBack = _rightBack;
    leftBack = _leftBack;

  }
 public void willsAutonMethod (){
        String currentState = "Start"; 

        switch (currentState){
            case "Start":
        rightFront.turny(0);
        leftFront.turny(0);
        rightBack.turny(0);
        leftBack.turny(0);
        rightFront.movey(0);
        leftFront.movey(0);
        rightBack.movey(0);
        leftBack.movey(0);
        gyro.reset();
        time = 0;
        currentState = "driving";
        break;
            
        case "driving":
        rightFront.turny(0);
        leftFront.turny( 0);
        rightBack.turny(0);
        leftBack.turny(0);
        rightFront.movey(-0.3);
        leftFront.movey(-0.3);
        rightBack.movey(-0.3);
        leftBack.movey(-0.3);
        if (time > 50) {
            rightFront.turny(0);
            leftFront.turny(0);
            rightBack.turny(0);
            leftBack.turny(0);            
            rightFront.movey(0);
            leftFront.movey(0);
            rightBack.movey(0);
            leftBack.movey(0);
            time = 0;
            currentState = "Rotating";
          }
            break;

        case "Rotating":
        rightFront.turny(90);
        rightFront.movey(-0.1);
        leftFront.turny(90);
        leftFront.movey(-0.1);
        rightBack.turny(0);
        rightBack.movey(-0.1);
        leftBack.turny(0);
        leftBack.movey(-0.1);
      }
    }
  }