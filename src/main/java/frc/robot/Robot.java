// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  boolean zeroMode = false;
  XboxController controller = new XboxController(0);
  Dashboard dashboard = new Dashboard(); 
  SparkMax liftMotor = new SparkMax(45, MotorType.kBrushless);
  XboxController operatorController = new XboxController(1);

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  SwerveModule rightFront = new SwerveModule(1,-134.8564-180,33,4,zeroMode);
  SwerveModule leftFront = new SwerveModule(0,66.3065-180,14,6,zeroMode);
  SwerveModule rightBack = new SwerveModule(2,64.7032-180,19,16,zeroMode);
  SwerveModule leftBack = new SwerveModule(3,85.9213-180,10,11,zeroMode);
  reeftoplayertoprocessor willsClass = new reeftoplayertoprocessor(rightFront, leftFront, rightBack, leftBack);
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
  
     
  public Robot() {
  }
  public void robotInit() {
  CameraServer.startAutomaticCapture(); 
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timerAuton.restart();
    gyro.reset();
    time = 0;
    
  }
String test = "start";
  @Override
  public void autonomousPeriodic() {
   test = willsClass.mattsMethod(test);

    //willsClass.willsAutonMethod();
    //PlayerToReef(time);
    time++;
    System.out.println("master" + test + time);


   /* double speedX = 0.0;
    double speedY = -0.1;
/*neg Y is forward
 * neg X is left
 */


/*if (timerAuton.get() < 2){
  speedX = 0.0;
  speedY = -0.1;
  } else if (timerAuton.get() < 4 && timerAuton.get() > 2) {
  speedX = 0.1;
  speedY = 0.0; 
  } else if (timerAuton.get() < 6 && timerAuton.get() > 4) {
    speedX = 0.0;
    speedY = 0.1; 
  }String currentState = "Start";
 
public void autonState(int time) {
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
rightFront.movey(-0.1);
leftFront.movey(-0.1);
rightBack.movey(-0.1);
leftBack.movey(-0.1);
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
}
break; else if (timerAuton.get() < 8 && timerAuton.get() > 6) {
    speedX = -0.1;
    speedY = 0.0;
  } else if (timerAuton.get() > 8) {
    timerAuton.restart();
  }
  else {
    speedX = 0.0; public PlayerToReef() {
        String State = "start";
        int time = 0;
        switch (State) {
            case "start":
                driveBase

        }
        




    }
/*ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(speedY,speedX,0.5,gyro.getRotation2d());
SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);

moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()))400
leftFront.turny(mod
  time = 0;leStates[1].angle.getDegrees());
leftFront.movey(moduleStates[1].speedMetersPerSecond/2);
rightBack.turny(moduleStates[2].angle.getDegrees());
rightBack.movey(moduleStates[2].speedMetersPerSecond/2);
leftBack.turny(moduleStates[3].angtime++;le.getDegrees());
leftBack.movey(moduleStates[3].speedMetersPerSecond/2);*/
  }

String currentState = "Start";
 
public void autonState(int time) {
  //willsClass.willsAutonMethod();
 //Will's case code
  switch (currentState) { 
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

case "turning":
rightFront.turny(0);
leftFront.turny( 0);
rightBack.turny(0);
leftBack.turny(0);
rightFront.movey(-0.4);
leftFront.movey(-0.1);
rightBack.movey(-0.1);
leftBack.movey(-0.1);
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
}
break; 


      case "turn1":
        time = 0;
        rightFront.turny(45);
        leftFront.turny( 135);
        rightBack.turny(135);
        leftBack.turny(45);
        rightFront.movey(0.1);
        leftFront.movey(0.1);
        rightBack.movey(-0.1);
        leftBack.movey(-0.1);
        if (time > 50){
          time = 0;
          rightFront.movey(0);
          leftFront.movey(0);
          rightBack.movey(0);
          leftBack.movey(0);
        }  
    }
  
  }


  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() { 
    dashboard.updateDashboard();

    if (zeroMode){
      System.out.println(
        rightFront.getRotation() 
        +", " + leftFront.getRotation()
        +", " + rightBack.getRotation()
        +", " + leftBack.getRotation()
      );
      return;
    } 
 
   
    if (controller.getYButton()){
      gyro.reset();
    }  
      
    ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),gyro.getRotation2d() );
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);

  moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
  moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
  moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
  moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));
  dashboard.updateDashboardSwerveModules(leftFront, rightFront, leftBack, rightBack); 

    rightFront.turny(moduleStates[0].angle.getDegrees());
    rightFront.movey(moduleStates[0].speedMetersPerSecond/2);
    leftFront.turny(moduleStates[1].angle.getDegrees());
    leftFront.movey(moduleStates[1].speedMetersPerSecond/2);
    rightBack.turny(moduleStates[2].angle.getDegrees());
    rightBack.movey(moduleStates[2].speedMetersPerSecond/2);
    leftBack.turny(moduleStates[3].angle.getDegrees());
    leftBack.movey(moduleStates[3].speedMetersPerSecond/2);

    
    double hsTargetspeed = MathUtil.clamp(operatorController.getLeftY(), -0.3, 0.3);
    liftMotor.set(hsTargetspeed);
    System.out.println(hsTargetspeed);
  
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
