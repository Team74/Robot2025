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
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  boolean zeroMode = false;
  boolean oldDriveBase = false;

  XboxController controller = new XboxController(0);
  Dashboard dashboard = new Dashboard(); 
  SparkMax liftMotor = null;
  XboxController operatorController = new XboxController(1);

  // Competition Bot and Old Base
  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  limeLightTest limelightcam = new limeLightTest(gyro);
  AnalogInput stringThingInput;
  AnalogPotentiometer stringThing;

  SwerveModule rightFront;
  SwerveModule leftFront;
  SwerveModule rightBack;
  SwerveModule leftBack;
  SwerveModule[] moduleList;
  StartToReef startToReef;
  driveForwardAuton driveForward;

  String autonState = "S2R";

  Translation2d frontRight = new Translation2d(0.33655, -0.33655); 
  Translation2d frontLeft = new Translation2d(0.33655, 0.33655); 
  Translation2d backRight = new Translation2d(-0.33655, -0.33655); 
  Translation2d backLeft = new Translation2d(-0.33655, 0.33655); 
  private final Timer timerAuton = new Timer();

  reeftoplayertoprocessor willsClass;
  SwerveDriveKinematics kinematics;

  Servo outtakeServo = new Servo(0);
  //Limit switch for the bottom of the lift
  //todo: rename
  DigitalInput limitSensorBottom = new DigitalInput(8);
  DigitalInput limitSensorTop = new DigitalInput(9);

  int time = 0;  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
     
  public Robot() {
    moduleList = new SwerveModule[4];

    if (!oldDriveBase) {
      // competition base CAN IDs
      rightFront = new SwerveModule(1,-134.8564,
          33,4,
          zeroMode,oldDriveBase);
      leftFront = new SwerveModule(0,66.3065,
          14,6,
          zeroMode,oldDriveBase);
      rightBack = new SwerveModule(2,64.7032,
          19,16,
          zeroMode,oldDriveBase);
      leftBack = new SwerveModule(3,85.9213,
          10,11,
          zeroMode,oldDriveBase);

      liftMotor = new SparkMax(12, MotorType.kBrushed);
      stringThingInput = new AnalogInput(0);
      stringThing = new AnalogPotentiometer(stringThingInput, 1, 0);

      
    } else {
      // old drive base CAN IDs
 
      rightFront = new SwerveModule(1,353,
          20,2,
          zeroMode,oldDriveBase);     
      leftFront = new SwerveModule(0,68,
          12,17,
          zeroMode,oldDriveBase);
      rightBack = new SwerveModule(2,358,
          14,32,
          zeroMode,oldDriveBase);
      leftBack = new SwerveModule(3,241-180,
          29,15,
          zeroMode,oldDriveBase);
    }
    moduleList[0] = rightFront;
    moduleList[1] = rightBack;
    moduleList[2] = leftFront;
    moduleList[3] = leftBack;

   // willsClass = new reeftoplayertoprocessor(rightFront, leftFront, rightBack, leftBack);
    kinematics = new SwerveDriveKinematics(frontRight, frontLeft, backRight, backLeft);
    //startToReef = new StartToReef(moduleList, liftMotor, outtakeServo);
    // driveForward = new driverForwardAuton(moduleList, liftMotor, kinematics, gyro);
  }
  
  public void robotInit() {
    CameraServer.startAutomaticCapture(); 
    time = 0;

    for(int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }
  
  @Override
  public void robotPeriodic(){

    
  }

  public void autonomousInit(){
    //startToReef.state = "start";
    time = 0;
    gyro.reset();
  }


String test = "start";
  @Override
  public void autonomousPeriodic() {

    autonState(time);
     switch (autonState) {
        
        case "S2R":
        //startToReef.RunS2R(time);
      }
      time ++;
    
    
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

case "driving":
rightFront.turny(0);
leftFront.turny( 0);
rightBack.turny(0);
leftBack.turny(0);
rightFront.movey(0.3);
leftFront.movey(0.3);
rightBack.movey(0.3);
leftBack.movey(0.3);
if (time > 100) {
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


      case "turning":
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

    System.out.println(stringThing.get());

    dashboard.updateDashboard();
    //limelightcam.LimeTest();
    double trackSide = 0;
    double trackTurn = 0; 
    double trackPush = 0;
if (controller.getLeftTriggerAxis() > 0.1){
  RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
            for (RawFiducial fiducial : fiducials) {
                    int id = fiducial.id;
                    double txnc = fiducial.txnc;
                    double tync = fiducial.tync;
                    double ta = fiducial.ta;
                    double distToCamera = fiducial.distToCamera;
                    double distToRobot = fiducial.distToRobot;
                    double ambiguity = fiducial.ambiguity; 
                System.out.println("Tag: " + id);
            }
          }
  trackSide = limelightcam.LimeTest();
  trackTurn = limelightcam.ReefCenter(); 
  trackPush = limelightcam.ReefPush();

    if (zeroMode){
      System.out.println(
        "RF:" + rightFront.getRotation() 
        +", LF:" + leftFront.getRotation()
        +", RB:" + rightBack.getRotation()
        +", LB:" + leftBack.getRotation()
      );
      return;
    } 
  
    if (controller.getYButton()){
      gyro.reset();
    }  
    
    if (controller.getXButton()){
      System.out.println(gyro.getAngle());
    }  
    ChassisSpeeds control;
    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam.CanSee()) {
      control = ChassisSpeeds.fromFieldRelativeSpeeds(trackPush*1, trackSide*1, trackTurn*1, new Rotation2d(0));
    } else {         
      control = ChassisSpeeds.fromFieldRelativeSpeeds(controller.getLeftY(), controller.getLeftX(), controller.getRightX(), gyro.getRotation2d() );
    }

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);

  moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
  moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
  moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
  moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));
  dashboard.updateDashboardSwerveModules(leftFront, rightFront, leftBack, rightBack); 
  
  rightFront.turny(moduleStates[0].angle.getDegrees());
  leftFront.turny(moduleStates[1].angle.getDegrees());
  rightBack.turny(moduleStates[2].angle.getDegrees());
  leftBack.turny(moduleStates[3].angle.getDegrees());

  if (controller.getRightBumperButton())  {
    rightFront.movey(moduleStates[0].speedMetersPerSecond);
    leftFront.movey(moduleStates[1].speedMetersPerSecond);
    rightBack.movey(moduleStates[2].speedMetersPerSecond);
    leftBack.movey(moduleStates[3].speedMetersPerSecond);

  }else {
    rightFront.movey(moduleStates[0].speedMetersPerSecond/2);
    leftFront.movey(moduleStates[1].speedMetersPerSecond/2);
    rightBack.movey(moduleStates[2].speedMetersPerSecond/2);
    leftBack.movey(moduleStates[3].speedMetersPerSecond/2); 
  }
  
    // liftMotor is only instantiated for competition base
    double hsTargetspeed = 0;

    if (liftMotor != null) {
      if (operatorController.getRightBumperButton()) {
        hsTargetspeed = MathUtil.clamp(operatorController.getLeftY()*-1, -0.5, 0.5);
      } else {
        hsTargetspeed = MathUtil.clamp(operatorController.getLeftY()*-1, -1.0, 1.0);
      }
      // System.err.println(!limitSensorBottom.get() + " " + !limitSensorTop.get() + " " + MathUtil.applyDeadband(operatorController.getLeftY(), 0.02));
      // //System.err.println(!limitSensorBottom.get() + " " + + MathUtil.applyDeadband(operatorController.getLeftY(), 0.02));
      //If the limit switch is triggered and control stick is down then stop!
      if (!limitSensorBottom.get() && MathUtil.applyDeadband(operatorController.getLeftY(), 0.02) > 0) {
        hsTargetspeed = 0;

        System.out.println("Bottom Limit Hit");
      } 
      //If the limit switch is triggered and control stick is down then stop!
      if (!limitSensorTop.get() && MathUtil.applyDeadband(operatorController.getLeftY(), 0.02) < 0) {
        hsTargetspeed = 0;

        System.out.println("Top Limit Hit");
      } 

      liftMotor.set(hsTargetspeed);

    }

    // outtakeServo is only instantiated for competition base
    if (outtakeServo != null) {
      // servo has 180 degree range
      double outtakeAngle = 180.0;
      if (operatorController.getAButton()) {
        outtakeAngle = 0.0;
      }
      outtakeServo.set(outtakeAngle / 180.0);
    }
    
    
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
