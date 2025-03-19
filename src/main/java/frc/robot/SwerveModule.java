package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveModule {
    int driveMotorID;
    SparkMax driveMotor;
    PIDController pid;
    SparkMax turningMotor;
    int turningMotorID;
    int encoderPort;
    double encoderOffset;
    DutyCycleEncoder encoder;
    AnalogEncoder encoderA;
    Boolean oldDriveBase = false;
boolean zeroMode = false;

public static final int kDrivingMotorCurrentLimit = 50; // amps
public static final int kTurningMotorCurrentLimit = 20; // amps
//From https://swervedrivespecialties.com/collections/kits/products/mk3-swerve-module
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 14 teeth on the bevel pinion
public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians

public static final int kEncoderCPR = 42;
public static final double kGearRatio = 8.1;
public static final double kEncoderDistanceConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(kGearRatio));
public static final double kEncoderVelocityConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(60*kGearRatio));

double turningFactor = kTurningEncoderPositionFactor;

    SwerveModule(int initialEncoderPort, 
                double initialEncoderOffset, 
                int initialTurningMotorID, 
                int initialDriveMotorID,
                boolean _zeroMode,
                boolean initialoldDriveBase) {
        
        encoderPort = initialEncoderPort;
        encoderOffset = initialEncoderOffset;
        turningMotorID = initialTurningMotorID;
        driveMotorID = initialDriveMotorID;
        oldDriveBase = initialoldDriveBase;
        zeroMode = _zeroMode;

        if(zeroMode){
            encoderOffset = 0.0;
           initialEncoderOffset = 0.0;
        }

        if (oldDriveBase){
            encoderA = new AnalogEncoder(initialEncoderPort,360.0,initialEncoderOffset);
        }
        else {

            encoder = new DutyCycleEncoder(initialEncoderPort,360.0,initialEncoderOffset-180.0);
        }

        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);

        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kDrivingMotorCurrentLimit);
        
        drivingConfig.encoder
                .positionConversionFactor(kEncoderDistanceConversionFactor) // meters
                .velocityConversionFactor(kEncoderVelocityConversionFactor); // meters per second
        
        turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(kTurningMotorCurrentLimit);
        
        turningConfig.absoluteEncoder
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0); //q radians per second

        driveMotor.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (oldDriveBase){
            pid = new PIDController(0.0023889, 0, 0);
            pid.enableContinuousInput(-180.0, 180.0);
        }
        else {
            pid = new PIDController(0.0025+0.0023889, 0, 0);
            pid.enableContinuousInput(-180.0, 180.0);
        }        
    }
    
    double returnRotation(){
        if (oldDriveBase) {
            // return encoderA.get() - 180 + encoderOffset;
            return encoderA.get();
        }
        return encoder.get() - 180 + encoderOffset;
    }
    
    double getRotation() {
        if (oldDriveBase) {
            if (encoderA.get() < 180) {
                return ((encoderA.get()))*-1;
            } else {
                return (encoderA.get())-(2*(encoderA.get()-180));
            }
        }
        if (zeroMode) {
            return encoder.get();
        }
        else {
            return encoder.get()-180;
        }
    }

    void turny(double targetAngle){
        double currentPosition = getRotation();
        double targetSpeed = pid.calculate(currentPosition,targetAngle);
        //if (turningMotorID == 12) {
        //    System.out.println("cp: " + currentPosition + "er: " + pid.getError() + "ta: "+ targetAngle);
        //}
        targetSpeed = MathUtil.clamp(targetSpeed, -0.75, 0.75);
        turningMotor.set(targetSpeed);
    }
    void movey(double targetSpeed){
        targetSpeed = MathUtil.clamp(targetSpeed, -1.0, 1.0);
        driveMotor.set(targetSpeed*-1);
    }
    void driveMotors (
        SwerveModule rightFrontMot, double rightFrontSpd,
        SwerveModule leftFrontMot, double leftFrontSpd, 
        SwerveModule rightBackMot, double rightBackSpd, 
        SwerveModule leftBackMot, double leftBackSpd
    ) {
        rightFrontMot.movey(rightFrontSpd);
        leftFrontMot.movey(leftFrontSpd);
        rightBackMot.movey(rightBackSpd);
        leftBackMot.movey(leftBackSpd);


    }

    public SwerveModulePosition getPosition() {
        //System.out.println("Encoder" + driveMotor.getEncoder().getPosition() + " Position" + encoder.get());
        if (oldDriveBase){
            return new SwerveModulePosition(encoderA.get(), new Rotation2d(encoderA.get()));
        }
        else {
            return new SwerveModulePosition(driveMotor.getEncoder().getPosition(), new Rotation2d(encoder.get()));
        }
        
    }

    // public void setDesiredState(SwerveModuleState desiredState) {
    //     var encoderRotation = new Rotation2d(getRotation());
    
    //     // Optimize the reference state to avoid spinning further than 90 degrees
    //     desiredState.optimize(encoderRotation);
    
    //     // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    //     // direction of travel that can occur when modules change directions. This results in smoother
    //     // driving.
    //     desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();
    
    //     // Calculate the drive output from the drive PID controller.
    //     final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);
    
    //     final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    
    //     // Calculate the turning motor output from the turning PID controller.
    //     final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());
    
    //     final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    
    //     driveMotor.setVoltage(driveOutput);
    //     m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    //   }
}


