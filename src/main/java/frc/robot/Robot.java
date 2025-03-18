// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.GotoPose.FieldPose;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.driveTrain.ShortcutType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import com.ctre.phoenix6.hardware.TalonFX; 
/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  boolean oldDriveBase = false;

  XboxController controller = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  Dashboard dashboard = new Dashboard(); 
  double targetAngleArm = 0;
  double armOffset = 0;
  Field2d m_field = new Field2d();

  limeLightTest limelightcam;
  driveTrain driveTrain;
  LimelightHelpers LimeHelp;

  StartToReef startToReef;
  driveForwardAuton driveForward;

  Auton_1P_SetUp auton_SetUp;
  AutonLeft_2P left_2p;
  AutonMiddle_2P middle_2P;
  AutonMiddle_Basic auton_Basic;
  AutonDriveForward autonDriveForward;

  UsbCamera fCamera;
  UsbCamera rCamera;
  //Joystick joy1Joystick = new Joystick(0);
  NetworkTableEntry ftNetworkTableEntry;

  reeftoplayertoprocessor willsClass;

  DigitalInput limitSensorBottom = new DigitalInput(8);
  DigitalInput armLimitTop = new DigitalInput(7);
  DigitalInput stringLiftLimit = new DigitalInput(9);

  GotoPose gotoPose;

  int time = 0;
  int intakeTime = 0;

  double currentAngle;
  /*private static final String driveForwardAuton = "Default_Auton";
  private static final String auAmp_2P = "Amp_2_Piece";
  private static final String auSource_2P = "Source_2_Piece";
  private static final String auShootMove = "Shoot_Move";
  private static final String auCenter_2P = "Center_2_Piece";
  private static final String auCenter_3P = "Center_3_Piece";*/

  DriverStation.Alliance alliancecolor = DriverStation.getAlliance().get();
  private static final String auto_AutonMiddle_basic = "Middle_Basic";
  private static final String auto_AutonMiddle_2P = "Middle_2P";
  private static final String auto_AutonLeft_2P = "Left_2P";
  private static final String auto_DriveTowardDriver = "DriveTowardDriver";
  private static final String auto_Auton_1P_SetUp = "Auton_1P_SetUp";

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;


  public Robot() {
    // fCamera = CameraServer.startAutomaticCapture(0);
    // rCamera = CameraServer.startAutomaticCapture(2);
    // ftNetworkTableEntry = NetworkTableInstance.getDefault().getTable("").getEntry("frontCamera");
    driveTrain = new driveTrain(dashboard, alliancecolor);
    LimeHelp = new LimelightHelpers();
    auton_SetUp = new Auton_1P_SetUp(driveTrain, limelightcam, LimeHelp);
    auton_Basic = new AutonMiddle_Basic(driveTrain, limelightcam, LimeHelp);
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    left_2p = new AutonLeft_2P(driveTrain, limelightcam);
    autonDriveForward = new AutonDriveForward(driveTrain, limelightcam);
    gotoPose = new GotoPose(driveTrain);
    limelightcam = new limeLightTest(driveTrain);

    m_chooser.setDefaultOption("Default Auto", auto_AutonMiddle_basic);
    m_chooser.addOption("Middle_1P", auto_AutonMiddle_basic);
    m_chooser.addOption("auton_1P_SetUp", auto_Auton_1P_SetUp);
    m_chooser.addOption("Middle_2P", auto_AutonMiddle_2P);
    m_chooser.addOption("Left_2P", auto_AutonLeft_2P);
    m_chooser.addOption("DriveTowardDriver", auto_DriveTowardDriver);

    SmartDashboard.putData("Auto choices", m_chooser);

    
  }

  public void robotInit() {
     var m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),

            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),

            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    m_field = new Field2d();
    dashboard.updatefielddata (m_field); 

    m_field.getObject("traj").setTrajectory(m_trajectory);
    // CameraServer.startAutomaticCapture(); 
    // CameraServer.startAutomaticCapture(); 
    time = 0;

    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {

  }

  public void autonomousInit() {
    time = 0;
    driveTrain.gyro.reset();
    autoState = new Object[] { "Starting", 0 };

    driveTrain.liftMotor.getEncoder().setPosition(0.0);
    //driveTrain.armMotor.getEncoder().setPosition(0.0);
    driveTrain.climbMotor.getEncoder().setPosition(0.0);

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);

    auton_Basic = new AutonMiddle_Basic(driveTrain, limelightcam, LimeHelp);
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    //auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    //auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    left_2p = new AutonLeft_2P(driveTrain, limelightcam);
    autonDriveForward = new AutonDriveForward(driveTrain, limelightcam);

    autoState = new Object[] { "Starting", 0 };
  }

  Object[] autoState = new Object[] { "Starting", 0 };

  @Override
  public void autonomousPeriodic() {
    if (driveTrain.zeroMode == true){
      System.out.println("System disabled"); 
      return; 
    }

    switch (m_autoSelected) {
      
      case auto_AutonMiddle_basic:
        
        autoState = auton_Basic.Run_2P(autoState, kDefaultPeriod);
      break;
      case auto_Auton_1P_SetUp:

        autoState = auton_SetUp.Run_2P(autoState, kDefaultPeriod);
      case auto_AutonMiddle_2P:
        
        autoState = middle_2P.Run_2P(autoState);
      break;
      case auto_AutonLeft_2P:
        
        autoState = left_2p.Run_2P(autoState);
      break;
      case auto_DriveTowardDriver:
        
        autoState = autonDriveForward.Run_2P(autoState);
      break;
      default:
        
        autoState = auton_Basic.Run_2P(autoState, kDefaultPeriod);
      break;
    }

    // autoState = auton_2p.Run_2P(autoState);
    // //autoState = auton_2p.Run_2P1(autoState, getPeriod());
    // //autoState = middle_2P.Run_2P(autoState);
    // //autoState = right_2p.Run_2P(autoState);

  }

  @Override
  public void teleopInit() {
    DriverStation.Alliance alliancecolor = DriverStation.getAlliance().get();
    DriverStation.Alliance redcolor = Alliance.Red;
    DriverStation.Alliance bluecolor = Alliance.Blue;
    if (alliancecolor == redcolor) {
     
    }
    if (alliancecolor == bluecolor) {
      // does something
    }

    desiredAngle = 0;
  }

  boolean hasPiece() {
    return !driveTrain.proxSensor.get();
  }
  @Override
  public void teleopPeriodic() {
    //camera switching for driver's view
    // if (controller.getAButton()){
    //   System.out.println("Setting rearcamera");
    //   ftNetworkTableEntry.setString(rCamera.getName());
    // } else if (controller.getBButton()) {
    //   System.out.println("Setting fCamera");
    //   ftNetworkTableEntry.setString(fCamera.getName());
    // }
    //auton climber feature
    
    dashboard.updateDashboard();

    if (driveTrain.zeroMode) {
      System.out.println(
          "RF:" + driveTrain.rightFront.getRotation()
              + ", LF:" + driveTrain.leftFront.getRotation()
              + ", RB:" + driveTrain.rightBack.getRotation()
              + ", LB:" + driveTrain.leftBack.getRotation());
      return;
    }

    //Button to resent the gyro
    if (controller.getYButton()) {
      driveTrain.gyro.reset();
    }

      //Button to resent the gyro
      if (operatorController.getRightBumperButton() && operatorController.getYButton()) {
        driveTrain.liftMotor.getEncoder().setPosition(0.0);
        driveTrain.armMotor.setPosition(0.0);
        driveTrain.climbMotor.getEncoder().setPosition(0.0);
      }
  

    //test controls
    if (controller.getXButton()) {
      //System.out.println(driveTrain.gyro.getAngle());
    }
   
    //Shortcut to align to the Apriltags
    if(controller.getBButton()) {
      //System.out.println("X value: " + tag.getRotation().getX() + "Y Value: " + tag.getRotation().getY() + "Z Value: " + tag.getRotation().getZ());
      System.out.println("odo X value: " + driveTrain.odometry.getEstimatedPosition().getRotation().getDegrees());
    }
    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam != null) {
      limelightcam.LimeTarget(getPeriod());
    } else {
      driveTrain.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),
          controller.getRightBumperButton(), controller.getLeftBumperButton());
    }

    var potval = driveTrain.potLift.get();
    var potArmVal = driveTrain.potArm.get();
    
    if(operatorController.getLeftBumperButton()) {
      System.out.println("potval: "+ potval + " ap: " + driveTrain.armMotor.getPosition().getValueAsDouble() + " LM:" + driveTrain.liftMotor.getEncoder().getPosition() + " potArmVal: " + potArmVal);
    }


    //Controls for the Scoring Arm
    if (driveTrain.armMotor != null) {
  
      double armClampSpeed = 0.5;
      Double armMotorSpeed = 0.0;

      if(operatorController.getLeftBumperButton()) {

        //player Station
         if(operatorController.getRightTriggerAxis() > 0) {
          armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.PLAYER);
        }

        //Trough
        if(operatorController.getAButton()) {
          armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L1);
        } 

        //Level 2
        if(operatorController.getBButton()) {
          armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L2);
        } 

        //Level 3
        if(operatorController.getXButton()) {
          armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L3);
        } 

        //Level 4
        if(operatorController.getYButton()) {
          armMotorSpeed = driveTrain.ShortCutArm(ShortcutType.L4);
        } 
        
      } 
      else {
        armMotorSpeed = MathUtil.applyDeadband(operatorController.getRightY(), 0.1) * armClampSpeed * 1;
      }
      driveTrain.armMotor.set(armMotorSpeed);

      
    }
  

    
    
    //Controls for the Scoring Lift
    if (driveTrain.liftMotor != null) {
      var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
      double liftMotorSpeed = 0;
      double liftClampSpeed = 1;

      liftMotorSpeed = MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * liftClampSpeed * -1;

      //lower limit
      if (!operatorController.getBButton() && driveTrain.liftMotor.getEncoder().getPosition() < 10 && liftMotorSpeed < 0){
        liftMotorSpeed = 0;
      }
      //Upper Limit
      if (!operatorController.getBButton() && driveTrain.liftMotor.getEncoder().getPosition() > 325 && liftMotorSpeed > 0){
        liftMotorSpeed = 0;
      }
     //System.out.println(driveTrain.liftMotor.getEncoder().getPosition());
      
      if(operatorController.getLeftBumperButton()) {
        //Human Player
        if(operatorController.getRightTriggerAxis() > 0) {
          liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.PLAYER);
          
        }

        //Trough
        if(operatorController.getAButton()) {
          liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L1);
        }
        
        //L2 
        if(operatorController.getBButton()) {
          liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L2);
        }
        
        //L3
        if(operatorController.getXButton()) {
          liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L3);
        }

        //L4
        if(operatorController.getYButton()) {
          liftMotorSpeed = driveTrain.ShortCutLift(ShortcutType.L4);
        }
      }
      else {
        liftMotorSpeed = MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * liftClampSpeed * -1;
      }

      if (!driveTrain.limitSensorBottom.get() && MathUtil.applyDeadband(operatorController.getLeftY(), 0.02) > 0) {
        liftMotorSpeed = 0;
        driveTrain.liftMotor.getEncoder().setPosition(0.0);

        System.out.println("Bottom Limit Hit");
      } 

      // if(!liftMotorPosition < 0 || liftMotorPosition > 40) {
      //   liftMotorSpeed = 0;
      //   System.out.println("Bottom Limit Hit");
      // }

      //System.out.println("liftMotorSpeed:" + liftMotorSpeed + "lm current: " + driveTrain.liftMotor.getOutputCurrent());
      driveTrain.liftMotor.set(liftMotorSpeed);
    }

    //Controls for the Climber
    if (driveTrain.climbMotor != null) {
      double climbSpeed = 0;
      double climbHeight = 0;

      climbHeight = driveTrain.climbMotor.getEncoder().getPosition();

      int pov = operatorController.getPOV();
      //System.out.println("pov: " + pov);

      if (pov == -1) {
        climbSpeed = 0;
      }
      else if (pov >= 315 || pov <= 45) {
        climbSpeed = -0.75;
    
      }
      else if (pov >= 135 && pov <= 225) {
        climbSpeed = 0.75;

      }

      //System.out.println("climbHeight: " + climbHeight);
      
      if (!operatorController.getBButton() && climbHeight > 115 && climbSpeed > 0){
          climbSpeed = 0;
      } 
      if (!operatorController.getBButton() && climbHeight < 5 && climbSpeed < 0){
        climbSpeed = 0;
      }

      driveTrain.climbMotor.set(climbSpeed);
    }


    //Outake and intake controls 
    //if statements for the algae (it goes faster to fling the algae away)
    //if else statements for the coral intake and outake
    if (!oldDriveBase) {
      double outTakeSpeed = 0;

      if (!operatorController.getLeftBumper()){

        if (MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) > 0 && operatorController.getAButton()){
          outTakeSpeed = -1;
          enableIntakeControls = buttonStates.Pressed;
        } else if (MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) > 0){
          outTakeSpeed = -0.5;
          enableIntakeControls = buttonStates.Pressed;
        }

        if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0 && operatorController.getAButton()){
          outTakeSpeed = 1;
          enableIntakeControls = buttonStates.Pressed;
        } else if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0){
          outTakeSpeed = 0.5;
          enableIntakeControls = buttonStates.Pressed;
        }

        if(MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) == 0 && MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) == 0) {
          enableIntakeControls = buttonStates.Released;
        }

      }
      currentAngle = driveTrain.potArm.get();
            if (currentAngle > 0 && currentAngle < 25) {
              if (operatorController.getRightTriggerAxis() > 0)
                outTakeSpeed = 0;  
            } 
                
            
            
            Timer.delay(0.02);  //

      //System.out.println("enableIntakeControls: " + enableIntakeControls);

      
      

      //Prox sensor lol

      if (hasPiece() == true) {
        if(intakeTime < 10 ){
          intakeTime++;
        }
        if(intakeTime >= 10 && intakeTime < 100 ){
          outTakeSpeed = 0; 
          intakeTime++;
        }
        System.out.println("Caught one!!!: intakeTime: " + intakeTime);
      }
      else {
        intakeTime = 0;
      }
      driveTrain.outTakeSet(outTakeSpeed);
    }

    m_field.setRobotPose(driveTrain.odometry.getEstimatedPosition());

    dashboard.updatefielddata (m_field);

  if(controller.getAButton()){
    //gotoPose.execute(FieldPose.BLUE_22);

    // if(desiredAngle == 0) {
       desiredAngle = 90;
    // }
    // if(desiredAngle == 90) {
    //   desiredAngle = 120;
    // }
    // if(desiredAngle == 120) {
    //   desiredAngle = 180;
    // }
    // if(desiredAngle == 180) {
    //   desiredAngle = -90;
    // }
    // if(desiredAngle == -90) {
    //   desiredAngle = 0;
    // }

    driveTrain.turnBotToAngle(desiredAngle);
  }
  intakeTime++;
}

int desiredAngle = 0;

buttonStates enableIntakeControls = buttonStates.NotPressed;

enum buttonStates {
  NotPressed, Pressed, Released
}
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
