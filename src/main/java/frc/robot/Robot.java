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

  Auton_2P auton_2p;

  reeftoplayertoprocessor willsClass;

  DigitalInput limitSensorBottom = new DigitalInput(8);
  DigitalInput limitSensorTop = new DigitalInput(9);

  int time = 0;

  public Robot() {
    driveTrain = new driveTrain(dashboard);
    auton_2p = new Auton_2P(driveTrain, limelightcam);

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
  }

  Object[] autoState = new Object[] { "Starting", 0 };

  @Override
  public void autonomousPeriodic() {

    autoState = auton_2p.Run_2P(autoState);

  }

  @Override
  public void teleopInit() {
    DriverStation.Alliance alliancecolor = DriverStation.getAlliance().get();
    DriverStation.Alliance redcolor = Alliance.Red;
    DriverStation.Alliance bluecolor = Alliance.Blue;
    if (alliancecolor == redcolor) {
      // does something
    }
    if (alliancecolor == bluecolor) {
      // does something
    }
  }

  @Override
  public void teleopPeriodic() {

    //what the sigma
    if (operatorController.getLeftBumperButton() && limelightcam.CanSee()) {

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
            driveTrain.drive(controller.getLeftX(), 0, Rotation, false);
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
    trackSide = limelightcam.LimeTest();
    trackTurn = limelightcam.ReefCenter();
    trackPush = limelightcam.ReefPush();

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
      driveTrain.liftMotor.getEncoder().setPosition(0.0);
      driveTrain.armMotor.getEncoder().setPosition(0.0);
    }

    //test controls
    if (controller.getXButton()) {
      System.out.println(driveTrain.gyro.getAngle());
    }

    //Shortcut to align to the Apriltags
    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam.CanSee()) {
      driveTrain.drive(trackPush, trackSide, trackTurn, controller.getRightBumperButton());
    } else {
      driveTrain.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),
          controller.getRightBumperButton());
    }
    System.out.println("ARM: " + driveTrain.armMotor.getEncoder().getPosition());

    //Controls for the Scoring Arm
    if (driveTrain.armMotor != null) {
      double armMotorSpeed = 0;
      double armClampSpeed = 0.6;

      armMotorSpeed = MathUtil.applyDeadband(operatorController.getRightY(), 0.1) * armClampSpeed;
      driveTrain.armMotor.set(armMotorSpeed);
    }
    System.out.println("LM:" + driveTrain.liftMotor.getEncoder().getPosition());
    
    //Controls for the Scoring Lift
    if (driveTrain.liftMotor != null) {
      double liftMotorSpeed = 0;
      double liftClampSpeed = 0.6;

      liftMotorSpeed = MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * liftClampSpeed;

      //lower limit
      if (driveTrain.liftMotor.getEncoder().getPosition() < 0){
        liftMotorSpeed = 0;
      } 
      //Upper Limit
      if (driveTrain.liftMotor.getEncoder().getPosition() > 444){
        liftMotorSpeed = 0;
      }

      driveTrain.liftMotor.set(liftMotorSpeed);      
    }


    //Controls for the Climber
    if (driveTrain.cageLift != null) {
      double cageSpeed = 0;
      double cageHeight = 0;

      cageHeight = driveTrain.cageLift.getEncoder().getPosition();

      int pov = operatorController.getPOV();
      System.out.println("pov: " + pov);

      if (pov == -1) {
        cageSpeed = 0;
      }
      else if (pov >= 315 || pov <= 45) {
        cageSpeed = 0.75;
    
      }
      else if (pov >= 135 && pov <= 225) {
        cageSpeed = -0.75;

      }

    System.out.println("cageHeight: " + cageHeight);
    driveTrain.cageLift.set(cageSpeed);

    if (cageHeight > 115) {
        cageSpeed = 0;
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
        outTakeSpeed = 0.5;
      }

      if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0 && operatorController.getAButton()){
        outTakeSpeed = -1;
      } else if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0){
        outTakeSpeed = -0.5;
      }
      
      driveTrain.outTakeSet(outTakeSpeed);

      //Limiting statements
      driveTrain.outTakeMotorInner.getOutputCurrent();
      System.out.println(driveTrain.outTakeMotorInner.getOutputCurrent());

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
