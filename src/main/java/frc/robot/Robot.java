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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  boolean zeroMode = false;
  boolean oldDriveBase = false;

  XboxController controller = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  Dashboard dashboard = new Dashboard(); 
  double targetAngleArm = 0;
  double armOffset = 0;
  Field2d m_field = new Field2d();

  limeLightTest limelightcam;
  driveTrain driveTrain;

  StartToReef startToReef;
  driveForwardAuton driveForward;

  AutonLeft_2P auton_2p;
  AutonMiddle_2P middle_2P;
  AutonMiddle_Basic right_2p;
  AutonDriveForward autonDriveForward;

  reeftoplayertoprocessor willsClass;

  DigitalInput limitSensorBottom = new DigitalInput(8);
  DigitalInput armLimitTop = new DigitalInput(7);
  DigitalInput stringLiftLimit = new DigitalInput(9);

  private final PIDController rotationPID;
  private final PIDController rangePID;

  int time = 0;

  /*private static final String driveForwardAuton = "Default_Auton";
  private static final String auAmp_2P = "Amp_2_Piece";
  private static final String auSource_2P = "Source_2_Piece";
  private static final String auShootMove = "Shoot_Move";
  private static final String auCenter_2P = "Center_2_Piece";
  private static final String auCenter_3P = "Center_3_Piece";*/

  DriverStation.Alliance alliancecolor = DriverStation.getAlliance().get();
  private static final String auto_AutonMiddle_1P = "Middle_1P";
  private static final String auto_AutonMiddle_2P = "Middle_2P";
  private static final String auto_AutonLeft_2P = "Left_2P";
  private static final String auto_DriveTowardDriver = "DriveTowardDriver";

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;

  public Robot() {

    driveTrain = new driveTrain(dashboard, alliancecolor);
    right_2p = new AutonMiddle_Basic(driveTrain, limelightcam);
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    autonDriveForward = new AutonDriveForward(driveTrain, limelightcam);

    limelightcam = new limeLightTest(driveTrain.gyro);

    m_chooser.setDefaultOption("Default Auto", auto_AutonMiddle_1P);
    m_chooser.addOption("Middle_1P", auto_AutonMiddle_1P);
    m_chooser.addOption("Middle_2P", auto_AutonMiddle_2P);
    m_chooser.addOption("Left_2P", auto_AutonLeft_2P);
    m_chooser.addOption("DriveTowardDriver", auto_DriveTowardDriver);

    SmartDashboard.putData("Auto choices", m_chooser);

    // Tune these PID values for your robot
    //rotationPID = new PIDController(0.0025+0.0023889, 0, 0);
    rotationPID = new PIDController(.05, 0.0, 0.001);
    rangePID = new PIDController(0.3, 0.0, 0.01);

    // Set tolerance for both controllers
    rotationPID.setTolerance(0.3); // 1 degree tolerance
    rangePID.setTolerance(0.01); // 5cm tolerance
    
    rotationPID.enableContinuousInput(-180.0, 180.0);
    rangePID.enableContinuousInput(-180.0, 180.0);
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
    CameraServer.startAutomaticCapture(); 
    CameraServer.startAutomaticCapture(); 
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
    driveTrain.armMotor.getEncoder().setPosition(0.0);
    driveTrain.climbMotor.getEncoder().setPosition(0.0);

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);

    right_2p = new AutonMiddle_Basic(driveTrain, limelightcam);
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    auton_2p = new AutonLeft_2P(driveTrain, limelightcam);
    autonDriveForward = new AutonDriveForward(driveTrain, limelightcam);

    autoState = new Object[] { "Starting", 0 };
  }

  Object[] autoState = new Object[] { "Starting", 0 };

  @Override
  public void autonomousPeriodic() {
    if (zeroMode == true){
      System.out.println("System disabled"); 
      return; 
    }

    switch (m_autoSelected) {
      
      case auto_AutonMiddle_1P:
        
        autoState = right_2p.Run_2P(autoState);
      break;
      case auto_AutonMiddle_2P:
        
        autoState = right_2p.Run_2P(autoState);
      break;
      case auto_AutonLeft_2P:
        
        autoState = auton_2p.Run_2P(autoState);
      break;
      case auto_DriveTowardDriver:
        
        autoState = autonDriveForward.Run_2P(autoState);
      break;
      default:
        
        autoState = auton_2p.Run_2P(autoState);
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
    //auton climber feature
    if (operatorController.getLeftBumperButton() && limelightcam != null && limelightcam.CanSee()) {

      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
      for (RawFiducial fiducial : fiducials) {
        int id = fiducial.id;
        double txnc = fiducial.txnc;
        double tync = fiducial.tync;
        double ta = fiducial.ta;
        double distToCamera = fiducial.distToCamera;
        double distToRobot = fiducial.distToRobot;
        double ambiguity = fiducial.ambiguity;

        double Rotation = limelightcam.calculaterotation(90.0);
        // System.out.println("rot: " + Rotation);
        if (id == 14 || id == 15 || id == 5 || id == 4) {
          if (controller.getLeftY() != 0) {
            driveTrain.drive(controller.getLeftX(), 0, Rotation, false, false);
          }
        }
      }
    }

    dashboard.updateDashboard();
    double trackSide = 0;
    double trackTurn = 0;
    double trackPush = 0;
    if (controller.getLeftTriggerAxis() > 0.1) {
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
      for (RawFiducial fiducial : fiducials) {
        int id = fiducial.id;
        double txnc = fiducial.txnc;
        double tync = fiducial.tync;
        double ta = fiducial.ta;
        double distToCamera = fiducial.distToCamera;
        double distToRobot = fiducial.distToRobot;
        double ambiguity = fiducial.ambiguity;
        //System.out.println("Tag: " + id);
      }
    }
    /*trackSide = limelightcam.LimeTest();
    trackTurn = limelightcam.ReefCenter();
    trackPush = limelightcam.ReefPush();*/

    if (zeroMode) {
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
        driveTrain.armMotor.getEncoder().setPosition(0.0);
        driveTrain.climbMotor.getEncoder().setPosition(0.0);
      }
  

    //test controls
    if (controller.getXButton()) {
      //System.out.println(driveTrain.gyro.getAngle());
    }

    //Shortcut to align to the Apriltags

    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam != null) {
      double ty = LimelightHelpers.getTY("limelight");
        
      // Get distance from target using 3D pose data
      double currentRange = LimelightHelpers.getTargetPose3d_CameraSpace("limelight").getZ();
      
      var tag = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
      double dist = tag.getTranslation().getNorm();
      System.out.println("X value: " + tag.getRotation().getX() + "Y Value: " + tag.getRotation().getY() + "Z Value: " + tag.getRotation().getZ());

      // Calculate control outputs
      double rotationOutput = rotationPID.calculate(ty, 0.0);
      double rangeOutput = rangePID.calculate(dist, 0.2);
      rangeOutput *= 4;

      rangeOutput = MathUtil.applyDeadband(rangeOutput, 0.1);

      // Apply control outputs to robot
      // Forward/backward movement for range control
      // Left/right movement is zero
      // Rotation is for horizontal alignment
      //driveTrain.driveLL(-rangeOutput, 0, -rotationOutput, true);
      driveTrain.driveLL(rangeOutput, 0, -rotationOutput, false, getPeriod());

    } else {
      driveTrain.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),
          controller.getRightBumperButton(), controller.getLeftBumperButton());
    }

    var potval = driveTrain.potLift.get();

    if(operatorController.getLeftBumperButton()) {
      System.out.println("potval: "+ potval);
    }


    //Controls for the Scoring Arm
    if (driveTrain.armMotor != null) {
      var armPosition = driveTrain.armMotor.getEncoder().getPosition();
      double armClampSpeed = 0.6;
      Double armMotorSpeed =0.0;

      armMotorSpeed = MathUtil.applyDeadband(operatorController.getRightY(), 0.1) * armClampSpeed * 1;

      // if(armPosition > 353 && armPosition < 553) {
      //   armMotorSpeed = 0;
      // }

      if(operatorController.getLeftBumperButton()) {
        System.out.println("armPosition: " + armPosition);

        //Human Player (this mean player station or processer???)
        //37.64
        if(operatorController.getRightTriggerAxis() > 0) {
          if(armPosition >= 0 && armPosition < 425.0636) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 18) {
            armMotorSpeed = -0.5;
          }
        }

        //Trough
        if(operatorController.getAButton()) {
          if(armPosition >= 0 && armPosition < 540.9) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 545.9) {
            armMotorSpeed = -0.5;
          }
        }
        
        //L2
        if(operatorController.getBButton()) {
          if(armPosition >= 0 && armPosition < 540.9) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 545.9) {
            armMotorSpeed = -0.5;
          }
        }
        
        //L3
        if(operatorController.getXButton()) {
          if(armPosition >= 0 && armPosition < 335.42) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 340) {
            armMotorSpeed = -0.5;
          }
        }

        //L4
        //375.5
        //lm: 541.60
        if(operatorController.getYButton()) {
          if(armPosition >= 0 && armPosition < 333.59) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 338) {
            armMotorSpeed = -0.5;
          }
        }
      }
      //Arm protecton
      /*if (driveTrain.armMotor != null){
        if (armLimitTop.get() == true  && operatorController.getRightY() < 0){
          armMotorSpeed = 0.0;
        }
      }

      if (driveTrain.liftMotor != null){
        
        if (stringLiftLimit.get() ==  0 && operatorController.getRightY() > 0){
        armMotorSpeed = 0.0;
        }
      }
      driveTrain.armMotor.set(armMotorSpeed);
    */
    }
  

    //System.out.println("LM:" + driveTrain.liftMotor.getEncoder().getPosition());
    
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
//System.out.println("liftMotorPosition: " + liftMotorPosition);

        //Human Player
        if(operatorController.getRightTriggerAxis() > 0) {
          if(liftMotorPosition >= 5 && liftMotorPosition < 146.28) {
            liftMotorSpeed = 1;
          }
          if(liftMotorPosition > 271) {
            liftMotorSpeed = -1;
          }
        }

        //Trough
        if(operatorController.getAButton()) {
          if(liftMotorPosition >= 5 && liftMotorPosition < 15) {
            liftMotorSpeed = 1;
          }
          if(liftMotorPosition > 20) {
            liftMotorSpeed = -1;
          }
        }
        
        //L2 
        //Arm:540.9
        //lm: 18
        if(operatorController.getBButton()) {
          if(liftMotorPosition >= 5 && liftMotorPosition < 6) {
            liftMotorSpeed = 1;
          }
          if(liftMotorPosition > 11) {
            liftMotorSpeed = -1;
          }
        }
        
        //L3
        if(operatorController.getXButton()) {
          if(liftMotorPosition >= 5 && liftMotorPosition < 0) {
            liftMotorSpeed = 1;
          }
          if(liftMotorPosition > 1) {
            liftMotorSpeed = -1;
          }
        }

        //L4
        if(operatorController.getYButton()) {
          if(liftMotorPosition >= 5 && liftMotorPosition < 263) {
            liftMotorSpeed = 1;
          }
          if(liftMotorPosition > 268) {
            liftMotorSpeed = -1;
          }
        }
      }

      // if (!driveTrain.limitSensorBottom.get() && MathUtil.applyDeadband(operatorController.getLeftY(), 0.02) > 0) {
      //   liftMotorSpeed = 0;
      //   driveTrain.liftMotor.getEncoder().setPosition(0.0);

      //   System.out.println("Bottom Limit Hit");
      // } 

      //System.out.println("liftMotorSpeed:" + liftMotorSpeed);
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
    driveTrain.climbMotor.set(climbSpeed);

    if (!operatorController.getBButton() && climbHeight > 115 && climbSpeed > 0){
        climbSpeed = 0;
    } 
    if (!operatorController.getBButton() && climbHeight < 5 && climbSpeed < 0){
      climbSpeed = 0;
    } 

  }


    //Outake and intake controls
    //if statements for the algae (it goes faster to fling the algae away)
    //if else statements for the coral intake and outake
    if (!oldDriveBase) {
      double outTakeSpeed = 0;

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
      
      driveTrain.outTakeSet(outTakeSpeed);

      //Prox sensor
      
      if (hasPiece() == true){
        //driveTrain.outTakeSet(outTakeSpeed*0.1);
        //System.out.println("Caught one!!!: " + (outTakeSpeed*0.1));
      }

    }
    m_field.setRobotPose(driveTrain.odometry.getEstimatedPosition());

    dashboard.updatefielddata (m_field);

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
