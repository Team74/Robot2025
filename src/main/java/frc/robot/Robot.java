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
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
boolean zeroMode = false;
XboxController controller = new XboxController(0);

AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

SwerveModule rightFront = new SwerveModule(1,46.7,33,4,zeroMode);
SwerveModule leftFront = new SwerveModule(0,-113.5,14,6,zeroMode);
SwerveModule rightBack = new SwerveModule(2,-120.8,19,16,zeroMode);
SwerveModule leftBack = new SwerveModule(3,-90.2,10,11,zeroMode);

Translation2d frontRight = new Translation2d(0.3032125, -0.314325); 
Translation2d frontLeft = new Translation2d(0.3032125, 0.314325); 
Translation2d backRight = new Translation2d(-0.3032125, -0.314325); 
Translation2d backLeft = new Translation2d(-0.3032125, 0.314325); 

SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontRight, frontLeft, backRight, backLeft);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (zeroMode){
    System.out.println(
    rightFront.getRotation() 
    +", " + leftFront.getRotation()
    +", " + rightBack.getRotation()
    +", " + leftBack.getRotation());
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

    rightFront.turny(moduleStates[0].angle.getDegrees());
    rightFront.movey(moduleStates[0].speedMetersPerSecond/2);
    leftFront.turny(moduleStates[1].angle.getDegrees());
    leftFront.movey(moduleStates[1].speedMetersPerSecond/2);
    rightBack.turny(moduleStates[2].angle.getDegrees());
    rightBack.movey(moduleStates[2].speedMetersPerSecond/2);
    leftBack.turny(moduleStates[3].angle.getDegrees());
    leftBack.movey(moduleStates[3].speedMetersPerSecond/2);
//thingy
  
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
