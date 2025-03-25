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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
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

  BotLED lights = new BotLED(3);
  boolean isRainbow = false;

  StartToReef startToReef;
  driveForwardAuton driveForward;

  
  AutonLeft_2PB autonLeft_2PB;
  AutonLeft_2P autonLeft_2P;
  Auton_1P_SetUp auton_SetUp;
  AutonRight_2P right_2p;
  AutonMiddle_2P middle_2P;
  AutonMiddle_Basic auton_Basic;
  AutonSide_Basic autonSide_Basic;
  AutonDriveForward autonDriveForward;

  UsbCamera fCamera;
  UsbCamera rCamera;
  // //Joystick joy1Joystick = new Joystick(0);
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

  DriverStation.Alliance alliancecolor;
  private static final String auto_AutonMiddle_basic = "Middle_Basic";
  private static final String auto_AutonSide_basic = "Side_Basic";
  private static final String auto_AutonMiddle_2PB = "Middle_2Basic";
  private static final String auto_AutonMiddle_2P = "Middle_2P";
  private static final String auto_AutonRight_2P = "Right_2P";
  private static final String auto_AutonLeft_2P = "Left_2P";
  private static final String auto_DriveTowardDriver = "DriveTowardDriver";
  private static final String auto_Auton_1P_SetUp = "Auton_1P_SetUp";

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;


  public Robot() {
    fCamera = CameraServer.startAutomaticCapture(0);
    rCamera = CameraServer.startAutomaticCapture(1);
     ftNetworkTableEntry = NetworkTableInstance.getDefault().getTable("").getEntry("frontCamera");

    //alliancecolor = DriverStation.getAlliance().get();


    fCamera = CameraServer.startAutomaticCapture(0);
    //rCamera = CameraServer.startAutomaticCapture(1);
    ftNetworkTableEntry = NetworkTableInstance.getDefault().getTable("").getEntry("frontCamera");
    
    driveTrain = new driveTrain(dashboard, alliancecolor);
    LimeHelp = new LimelightHelpers();
    auton_SetUp = new Auton_1P_SetUp(driveTrain, limelightcam, LimeHelp);
    auton_Basic = new AutonMiddle_Basic(driveTrain, limelightcam, LimeHelp);
    autonSide_Basic = new AutonSide_Basic(driveTrain, limelightcam, LimeHelp);
    
    autonLeft_2PB = new AutonLeft_2PB(driveTrain, limelightcam, LimeHelp);
    autonLeft_2P = new AutonLeft_2P(driveTrain, limelightcam, hasPiece());
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    right_2p = new AutonRight_2P(driveTrain, limelightcam, hasPiece());
    autonDriveForward = new AutonDriveForward(driveTrain, limelightcam);
    gotoPose = new GotoPose(driveTrain);
    limelightcam = new limeLightTest(driveTrain);

    m_chooser.setDefaultOption("Default Auto", auto_AutonMiddle_basic);
    m_chooser.addOption("Middle_basic", auto_AutonMiddle_basic);
    m_chooser.addOption("Side_basic", auto_AutonSide_basic);
    m_chooser.addOption("Left_2PB", auto_AutonMiddle_2PB);
    m_chooser.addOption("Left_2P", auto_AutonMiddle_2P);
    m_chooser.addOption("auton_1P_SetUp", auto_Auton_1P_SetUp);
    m_chooser.addOption("Middle_2P", auto_AutonMiddle_2P);
    m_chooser.addOption("Right_2P", auto_AutonRight_2P);
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
    //CameraServer.startAutomaticCapture(); 
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
    //driveTrain.gyro.reset();
    autoState = new Object[] { "Starting", 0 };

    driveTrain.liftMotor.getEncoder().setPosition(0.0);
    driveTrain.armMotor.setPosition(0.0);
    driveTrain.climbMotor.getEncoder().setPosition(0.0);

    for(int i = 0; i < 10; i++) {
      var armPosition = driveTrain.armMotor.getPosition().getValueAsDouble();
      armPosition = Math.rint(armPosition);
  
      if(armPosition > -4.8 && armPosition <= -5.1) {
        System.out.println("Moving arm out" + armPosition);
        var offset = driveTrain.pidArmToAngle(-5.0);
  
        driveTrain.armMotor.set(offset);
      }
      else {
        driveTrain.armMotor.setPosition(0.0);
        System.out.println("arm gtg. start auton");
        break;
      }
    }

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);

    auton_Basic = new AutonMiddle_Basic(driveTrain, limelightcam, LimeHelp);
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    //auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    //auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    right_2p = new AutonRight_2P(driveTrain, limelightcam, hasPiece());
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
        case auto_AutonMiddle_2PB:
          
          autoState = autonLeft_2PB.Run_2P(autoState, kDefaultPeriod);
        break;
        case auto_AutonLeft_2P:
          
          autoState = autonLeft_2P.Run_2P(autoState, kDefaultPeriod);
        break;
        case auto_AutonSide_basic:
          
        autoState = autonSide_Basic.Run_2P(autoState, kDefaultPeriod);
        break;
        case auto_Auton_1P_SetUp:
  
          autoState = auton_SetUp.Run_2P(autoState, kDefaultPeriod);
        case auto_AutonMiddle_2P:
          
          autoState = middle_2P.Run_2P(autoState);
        break;
        case auto_AutonRight_2P:
          
          autoState = right_2p.Run_2P(autoState, kDefaultPeriod);
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
  }

  boolean hasPiece() {
    return !driveTrain.proxSensor.get();
  }

  @Override
  public void teleopPeriodic() {
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
   
    //Shortcut to align to the Apriltags
    if(controller.getBButton()) {
      System.out.println("odo X value: " + driveTrain.gyro.getYaw());
    }

    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam != null) {
      limelightcam.LimeTarget(getPeriod());
    } else {
      driveTrain.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),
          controller.getRightBumperButton(), controller.getRightTriggerAxis() > 0);
    }

    if(operatorController.getLeftBumperButton()) {
      var potval = driveTrain.potLift.get();
      var potArmVal = driveTrain.potArm.get();
  
      System.out.println("potval: "+ potval + " ap: " + driveTrain.armMotor.getPosition().getValueAsDouble() + " LM:" + driveTrain.liftMotor.getEncoder().getPosition() + " potArmVal: " + potArmVal);
    }

    double armClampSpeed = 0.5;
    Double armMotorSpeed = 0.0;
    double liftMotorSpeed = 0;
    double liftClampSpeed = 1;

    //Controls for the Scoring Arm
    if (driveTrain.armMotor != null) {
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

      var armPosition = driveTrain.armMotor.getPosition().getValueAsDouble();

      //the arm is too far rotated and the system is saying to keep rotating
      if(armPosition < -100 && armMotorSpeed < 0) {
        armMotorSpeed = 0.0;
        System.out.println("Arm logical limit " + armPosition);
      }
      driveTrain.armMotor.set(armMotorSpeed);
    }
  
    //Controls for the Scoring Lift
    if (driveTrain.liftMotor != null) {
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

      var liftMotorPosition = driveTrain.potLift.get();

      //lower limit
      if (!operatorController.getBButton() && liftMotorPosition < 1 && liftMotorSpeed < 0){
        System.out.println("Logical Bottom Limit Hit");

        liftMotorSpeed = 0;
      }
      //Upper Limit
      if (!operatorController.getBButton() && liftMotorPosition > 60.5 && liftMotorSpeed > 0){
        System.out.println("Logical Upper Limit Hit");

        liftMotorSpeed = 0;
      }
      //Check physical bottom limit switch
      if (!driveTrain.limitSensorBottom.get() && MathUtil.applyDeadband(operatorController.getLeftY(), 0.02) > 0) {
        liftMotorSpeed = 0;
        driveTrain.liftMotor.getEncoder().setPosition(0.0);

        System.out.println("Bottom Limit Hit");
      } 
      driveTrain.liftMotor.set(liftMotorSpeed);
    }

    //Controls for the Climber
    if (driveTrain.climbMotor != null) {
      double climbSpeed = 0;
      double climbHeight = 0;

      climbHeight = driveTrain.climbMotor.getEncoder().getPosition();

      int pov = operatorController.getPOV();

      if (pov == -1) {
        climbSpeed = 0;
      }
      else if (pov >= 315 || pov <= 45) {
        climbSpeed = -0.75;
      }
      else if (pov >= 135 && pov <= 225) {
        climbSpeed = 0.75;
      }

      if (!operatorController.getBButton() && climbHeight > 115 && climbSpeed > 0){
          climbSpeed = 0;
      } 
      if (!operatorController.getBButton() && climbHeight < 5 && climbSpeed < 0){
        climbSpeed = 0;
      }

      driveTrain.climbMotor.set(climbSpeed);
    }

    //Outake and intake controls 
    if (!oldDriveBase) {
      double outTakeSpeed = 0;

      if (!operatorController.getLeftBumper()){
        if (MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) > 0 && operatorController.getAButton()){
          outTakeSpeed = -1;
        } else if (MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) > 0){
          outTakeSpeed = -0.5;
        }

        if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0 && operatorController.getAButton()){
          outTakeSpeed = 1;
        } else if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0){
          outTakeSpeed = 0.5;
        }
      }

      currentAngle = driveTrain.armMotor.getPosition().getValueAsDouble();

      //current angle greater than -82 (0 is straight down)
      //so if player, l1 or l2 and trying to eject... slow alot
      
      //System.out.println("ca: " + currentAngle);

      if (currentAngle > driveTrain.armPlayerPosition - 11 && operatorController.getLeftTriggerAxis() > 0) {
        outTakeSpeed = outTakeSpeed * 0.7;  
        System.out.println("slowing outtake");
      } 

      //if current angle less than l3 and intake button pressed... slow alot
      if (currentAngle < driveTrain.armL3Position+10 && operatorController.getRightTriggerAxis() > 0) {
          outTakeSpeed = outTakeSpeed * 0.7;  
          System.out.println("slowing int");
      } 
          
      //Prox sensor lol
      if (hasPiece() == true) {
        if(intakeTime < 3 ){
          intakeTime++;
        }
        if(intakeTime >= 3 && intakeTime < 100 ){
          outTakeSpeed = 0; 
          intakeTime++;
        }
      }
      else {
        intakeTime = 0;
      }
      driveTrain.outTakeSet(outTakeSpeed);
    }
    intakeTime++;

    m_field.setRobotPose(driveTrain.odometry.getEstimatedPosition());

    dashboard.updatefielddata (m_field);

    if(controller.getAButton()){
      double desiredAngle = 0d;
      
      //gotoPose.execute(FieldPose.BLUE_22);

      // if(desiredAngle == 0) {
      //  desiredAngle = 90;
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


      //LED controls???
      if(controller.getYButton()){
            isRainbow = !isRainbow;
          }

      if (!isRainbow) {
        if (hasPiece() == true) {// green
          lights.setColor(0, 255, 0);
          System.out.println("piece in");
        } 
      } else {
        lights.rainbow();
      }
    
      driveTrain.turnBotToAngle(desiredAngle);
    }
}

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    time = 0;
    time = 0;
  }

  @Override
  public void testPeriodic() {
    System.out.println("gryo: " + driveTrain.gyro.getYaw());

    if(time < 300) {
      var rot = driveTrain.getTurnBotToAngle(130);
      driveTrain.drive(0, 0, rot, false, false);
    }else {
      driveTrain.drive(0, 0, 0, false, false);

    }

    time++;
    }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
