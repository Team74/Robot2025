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
  AutonRight_2P right_2p;

  reeftoplayertoprocessor willsClass;

  DigitalInput proxSensor = new DigitalInput(4);
  DigitalInput limitSensorTop = new DigitalInput(9);

  int time = 0;

  DriverStation.Alliance alliancecolor = DriverStation.getAlliance().get();

  public Robot() {

    driveTrain = new driveTrain(dashboard, alliancecolor);
    right_2p = new AutonRight_2P(driveTrain, limelightcam);
    middle_2P = new AutonMiddle_2P(driveTrain, limelightcam);
    auton_2p = new AutonLeft_2P(driveTrain, limelightcam);

    limelightcam = new limeLightTest(driveTrain.gyro);
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

  }

  Object[] autoState = new Object[] { "Starting", 0 };

  @Override
  public void autonomousPeriodic() {

    autoState = auton_2p.Run_2P(autoState);
    //autoState = middle_2P.Run_2P(autoState);
    //autoState = right_2p.Run_2P(autoState);

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
    return !proxSensor.get();
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
        System.out.println("Tag: " + id);
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
      System.out.println(driveTrain.gyro.getAngle());
    }

    //Shortcut to align to the Apriltags
    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam != null && limelightcam.CanSee()) {
      driveTrain.drive(trackPush, trackSide, trackTurn, controller.getRightBumperButton(), controller.getLeftBumperButton());
    } else {
      driveTrain.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),
          controller.getRightBumperButton(), controller.getLeftBumperButton());
    }


    //System.out.println("ARM: " + armPosition);

    //Controls for the Scoring Arm
    if (driveTrain.armMotor != null) {
      var armPosition = driveTrain.armMotor.getEncoder().getPosition();

      double armMotorSpeed = 0;
      double armClampSpeed = 0.6;

      armMotorSpeed = MathUtil.applyDeadband(operatorController.getRightY(), 0.1) * armClampSpeed * -1;

      // if(armPosition > 353 && armPosition < 553) {
      //   armMotorSpeed = 0;
      // }

      if(operatorController.getLeftBumperButton()) {
        System.out.println("armPosition: " + armPosition);

        //Human Player
        //37.64
        if(operatorController.getRightTriggerAxis() > 0) {
          if(armPosition >= 0 && armPosition < 37.64) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 42) {
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
          if(armPosition >= 0 && armPosition < 335.42) {
            armMotorSpeed = 0.5;
          }
          if(armPosition > 340) {
            armMotorSpeed = -0.5;
          }
        }
      }

      driveTrain.armMotor.set(armMotorSpeed);
    }

    //System.out.println("LM:" + driveTrain.liftMotor.getEncoder().getPosition());
    
    //Controls for the Scoring Lift
    if (driveTrain.liftMotor != null) {
      var liftMotorPosition = driveTrain.liftMotor.getEncoder().getPosition();
      double liftMotorSpeed = 0;
      double liftClampSpeed = 1;

      liftMotorSpeed = MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * liftClampSpeed * -1;

      //lower limit
      if (!operatorController.getBButton() && driveTrain.liftMotor.getEncoder().getPosition() < 20 && liftMotorSpeed < 0){
        liftMotorSpeed = 0;
      }
      //Upper Limit
      if (!operatorController.getBButton() && driveTrain.liftMotor.getEncoder().getPosition() > 520 && liftMotorSpeed > 0){
        liftMotorSpeed = 0;
      }
     //System.out.println(driveTrain.liftMotor.getEncoder().getPosition());
      
      if(operatorController.getLeftBumperButton()) {
System.out.println("liftMotorPosition: " + liftMotorPosition);
        //Human Player
        if(operatorController.getRightTriggerAxis() > 0) {
          if(liftMotorPosition >= 0 && liftMotorPosition < 0) {
            liftMotorSpeed = 0.5;
          }
          if(liftMotorPosition > 1) {
            liftMotorSpeed = -0.5;
          }
        }

        //Trough
        if(operatorController.getAButton()) {
          if(liftMotorPosition >= 0 && liftMotorPosition < 18) {
            liftMotorSpeed = 0.5;
          }
          if(liftMotorPosition > 20) {
            liftMotorSpeed = -0.5;
          }
        }
        
        //L2 
        //Arm:540.9
        //lm: 18
        if(operatorController.getBButton()) {
          if(liftMotorPosition >= 0 && liftMotorPosition < 302) {
            liftMotorSpeed = 0.5;
          }
          if(liftMotorPosition > 307) {
            liftMotorSpeed = -0.5;
          }
        }
        
        //L3
        if(operatorController.getXButton()) {
          if(liftMotorPosition >= 0 && liftMotorPosition < 0) {
            liftMotorSpeed = 0.5;
          }
          if(liftMotorPosition > 1) {
            liftMotorSpeed = -0.5;
          }
        }

        //L4
        if(operatorController.getYButton()) {
          if(liftMotorPosition >= 0 && liftMotorPosition < 541.6) {
            liftMotorSpeed = 0.5;
          }
          if(liftMotorPosition > 545.0) {
            liftMotorSpeed = -0.5;
          }
        }
      }

      if (!driveTrain.limitSensorBottom.get() && MathUtil.applyDeadband(operatorController.getLeftY(), 0.02) > 0) {
        liftMotorSpeed = 0;
        driveTrain.liftMotor.getEncoder().setPosition(0.0);

        System.out.println("Bottom Limit Hit");
      } 

      System.out.println("liftMotorSpeed:" + liftMotorSpeed);
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
        outTakeSpeed = 1;
      } else if (MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) > 0){
        outTakeSpeed = 0.3;
      }

      if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0 && operatorController.getAButton()){
        outTakeSpeed = -1;
      } else if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0){
        outTakeSpeed = -0.3;
      }
      
      driveTrain.outTakeSet(outTakeSpeed);

      //Prox sensor
      
      if (hasPiece() == true){
        //driveTrain.outTakeSet(outTakeSpeed*0.1);
        System.out.println("Caught one!!!: " + (outTakeSpeed*0.1));
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
