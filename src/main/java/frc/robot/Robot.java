// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    if (controller.getYButton()) {
      driveTrain.gyro.reset();
    }

    if (controller.getXButton()) {
      System.out.println(driveTrain.gyro.getAngle());
    }

    if (controller.getLeftTriggerAxis() > 0.1 && limelightcam.CanSee()) {
      driveTrain.drive(trackPush, trackSide, trackTurn, controller.getRightBumperButton());
    } else {
      driveTrain.drive(controller.getLeftY(), controller.getLeftX(), controller.getRightX(),
          controller.getRightBumperButton());
    }

    double liftMotorSpeed = 0;

    if (driveTrain.armMotor != null) {
      if (MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) > 0) {
        liftMotorSpeed = 1;
      }

      if (MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) < 0) {
        liftMotorSpeed = -1;
      }
      liftMotorSpeed = operatorController.getLeftY() * 0.3;

      driveTrain.armMotor.set(liftMotorSpeed);
    }

    double cageSpeed = 0;
    double cageHeight = 0;

    if (driveTrain.cageLift != null) {

      cageHeight = driveTrain.cageLift.getEncoder().getPosition();

      int pov = operatorController.getPOV();
      System.out.println("pov: " + pov);

      if (pov == -1) {
        cageSpeed = 0;
      } else if (pov >= 315 || pov <= 45) {
        cageSpeed = 0.1;

      } else if (pov >= 135 && pov <= 225) {
        cageSpeed = -0.1;

      }
      System.out.println("cageHeight: " + cageHeight);

      driveTrain.cageLift.set(cageSpeed);
    }

    double outTakeSpeed = 0;

    if (!oldDriveBase) {

      if (MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), 0.1) > 0) {
        outTakeSpeed = 1;
      }

      if (MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.1) > 0) {
        outTakeSpeed = -1;
      }

      driveTrain.outTakeSet(outTakeSpeed);
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
