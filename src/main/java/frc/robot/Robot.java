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
public class Robot extends TimedRobot {
  boolean zeroMode = false;
  XboxController controller = new XboxController(0);
  Dashboard dashboard = new Dashboard();

  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  SwerveModule rightFront = new SwerveModule(1,48.6278,33,4,zeroMode);
  SwerveModule leftFront = new SwerveModule(0,-112.6435,13,44,zeroMode);
  SwerveModule rightBack = new SwerveModule(2,-105.9345,19,16,zeroMode);
  SwerveModule leftBack = new SwerveModule(3,-91.9409,10,11,zeroMode);

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

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    timerAuton.restart();
    gyro.reset();
    time = 0;

  }

  //@Override
  public void autonomousPeriodic1() {
    autonState(time);
    time++;
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
  } else if (timerAuton.get() < 8 && timerAuton.get() > 6) {
    speedX = -0.1;
    speedY = 0.0;
  } else if (timerAuton.get() > 8) {
    timerAuton.restart();
  }
  else {
    speedX = 0.0;
    speedY = 0.0;
}*/
/*ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(speedY,speedX,0.5,gyro.getRotation2d());
SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);

moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));

rightFront.turny(moduleStates[0].angle.getDegrees());
rightFront.movey(moduleStates[0].speedMetersPerSecond/2);
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
break;

        /*case "Rotating":
        rightFront.turny(90);/* */
        
      }
    }
  @Override

  public void autonomousPeriodic (){ 
    // Drive for 2 seconds

    /*if (m_timer.get() < 2.0) {
    rightFront.turny(0);
    rightFront.movey(-0.1);
    leftFront.turny(0);
    leftFront.movey(-0.1);
    rightBack.turny(0);
    rightBack.movey(-0.1);
    leftBack.turny(0);
    leftBack.movey(-0.1);
   } else {

      rightFront.turny(0);
      rightFront.movey(0);
      leftFront.turny(0);
      leftFront.movey(0);
      rightBack.turny(0);
      rightBack.movey(0);
      leftBack.turny(0);
      leftBack.movey(0);
    }

  }*/
  }
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (zeroMode){
      System.out.println(
        rightFront.getRotation() 
        +", " + leftFront.getRotation()
        +", " + rightBack.getRotation()
        +", " + leftBack.getRotation()
      );
      return;
    } 
  dashboard.updateDashboard();
  if (controller.getYButton()){
    gyro.reset();
  }  
      
  ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),gyro.getRotation2d() );
  SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);

  moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
  moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
  moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
  moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));
  dashboard.updateDashboardSwerveModules(moduleStates, leftFront, rightFront, leftBack, rightBack); 

  rightFront.turny(moduleStates[0].angle.getDegrees());
  rightFront.movey(moduleStates[0].speedMetersPerSecond/2);
  leftFront.turny(moduleStates[1].angle.getDegrees());
  leftFront.movey(moduleStates[1].speedMetersPerSecond/2);
  rightBack.turny(moduleStates[2].angle.getDegrees());
  rightBack.movey(moduleStates[2].speedMetersPerSecond/2);
  leftBack.turny(moduleStates[3].angle.getDegrees());
  leftBack.movey(moduleStates[3].speedMetersPerSecond/2);

  
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
