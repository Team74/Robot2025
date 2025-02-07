package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.BooleanLogEntry;
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

    SwerveModule(int initialEncoderPort, 
                double initialEncoderOffset, 
                int initialTurningMotorID, 
                int initialDriveMotorID,
                boolean zeroMode,
                boolean initialoldDriveBase) {
        if(zeroMode){
            initialEncoderOffset = 0;
        }
        encoderPort = initialEncoderPort;
        encoderOffset = initialEncoderOffset;
        turningMotorID = initialTurningMotorID;
        driveMotorID = initialDriveMotorID;
        oldDriveBase = initialoldDriveBase;
        if (oldDriveBase){
            encoderA = new AnalogEncoder(initialEncoderPort,360,initialEncoderOffset);
        }
        else {
            encoder = new DutyCycleEncoder(initialEncoderPort,360,initialEncoderOffset);
        }

        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        pid = new PIDController(0.0023889, 0, 0);
        pid.enableContinuousInput(-180, 180);
        SparkBaseConfig zeroCoast = new SparkMaxConfig();
        if (zeroMode){
            zeroCoast.idleMode(SparkBaseConfig.IdleMode.kCoast);
        } else {
            zeroCoast.idleMode(SparkBaseConfig.IdleMode.kBrake);
        }
        
    }
    
    double returnRotation(){
        if (oldDriveBase) {
            return encoderA.get() - 180 + encoderOffset;
        }
        return encoder.get() - 180 + encoderOffset;
    }
    
    double getRotation() {
        if (oldDriveBase) {
            return encoderA.get()-180;
        }
        return encoder.get()-180;
    }

    void turny(double targetAngle){
        double currentPosition = getRotation();
        double targetSpeed = pid.calculate(currentPosition,targetAngle);
        targetSpeed = MathUtil.clamp(targetSpeed, -0.5, 0.5);
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
    SwerveModule leftBackMot, double leftBackSpd){
        rightFrontMot.movey(rightFrontSpd);
        leftFrontMot.movey(leftFrontSpd);
        rightBackMot.movey(rightBackSpd);
        leftBackMot.movey(leftBackSpd);


    }


}


