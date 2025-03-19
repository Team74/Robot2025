package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Calendar;
import java.util.Date;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class driveTrain {
    boolean zeroMode = false;
    boolean oldDriveBase = false;
    
    public SwerveModule leftFront;
    public SwerveModule rightFront;
    public SwerveModule rightBack;
    public SwerveModule leftBack;

    //DigitalInput proxSensor = new DigitalInput(5);
    DigitalInput proxSensor = new DigitalInput(4);

    SwerveDriveKinematics kinematics;

   SwerveDrivePoseEstimator odometry;
    //SwerveDriveOdometry odometry;

    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    Dashboard dashboard;

    SparkMax liftMotor = null;
    TalonFX armMotor = null;
    SparkMax outTakeMotorOuter = null;
    SparkMax outTakeMotorInner = null;
    SparkMax climbMotor = null;
    PIDController pid;
    PIDController pidArm;
    double armSpeed;
    double currentAngleArm;


    PIDController pidLift;
    double liftSpeed;
    double targetLiftHeight;
    double currentHeightLift = 0;

    Calendar calendar = Calendar.getInstance();
    //DigitalInput limitSensorTop = new DigitalInput(7);
    DigitalInput limitSensorBottom = new DigitalInput(5);
    AnalogPotentiometer potLift = new AnalogPotentiometer(0,90, 0);
    AnalogPotentiometer potArm = new AnalogPotentiometer(1,90, 0);

    double powerMulti = 0.6;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, tol; //the PID loop doubles 

    DriverStation.Alliance alliancecolor;
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(10));

    ProfiledPIDController pidShortcutArm = new ProfiledPIDController(0.06, 0, 0, new TrapezoidProfile.Constraints(40, 70));
    ProfiledPIDController pidShortcutLift = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(40, 70));

    driveTrain(Dashboard dash, DriverStation.Alliance _alliancecolor) {
        gyro.reset();
        dashboard = dash;
        alliancecolor = _alliancecolor;

        if (!oldDriveBase) {
            // competition base CAN IDs
            //RF:45.88528614713215, LF:76.40100191002506, RB:-159.65606199140154, LB:-96.41761441044036
            leftFront = new SwerveModule(0,76.40100191002506, 6, 14, zeroMode,oldDriveBase);
            rightFront = new SwerveModule(2,45.88528614713215-180, 33,4, zeroMode,oldDriveBase);
            rightBack = new SwerveModule(3,-173.11333482783334-180-10, 10, 11, zeroMode,oldDriveBase);
            leftBack = new SwerveModule(1,-96.41761441044036-180, 19, 16, zeroMode,oldDriveBase);

            liftMotor = new SparkMax(46, MotorType.kBrushless);
            liftMotor.getEncoder().setPosition(0.0);
            SparkBaseConfig zeroCoast = new SparkMaxConfig();
            zeroCoast.idleMode(SparkBaseConfig.IdleMode.kBrake);
            liftMotor.configure(zeroCoast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
      
            armMotor = new TalonFX(3);

            TalonFXConfiguration toConfigure = new TalonFXConfiguration();
            CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

            m_currentLimits.SupplyCurrentLimit = 10; // Limit to 1 amps
            m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it
            m_currentLimits.StatorCurrentLimit = 10; // Limit stator to 20 amps
            m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

            toConfigure.CurrentLimits = m_currentLimits;

            armMotor.getConfigurator().apply(toConfigure);
            armMotor.setNeutralMode(NeutralModeValue.Brake);

            //armMotor.getEncoder().setPosition(0.0);
            zeroCoast.idleMode(SparkBaseConfig.IdleMode.kBrake);
            //armMotor.configure(zeroCoast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
            outTakeMotorOuter = new SparkMax(47, MotorType.kBrushed);
            outTakeMotorInner = new SparkMax(45, MotorType.kBrushed);
            outTakeMotorOuter.configure(zeroCoast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            outTakeMotorInner.configure(zeroCoast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            climbMotor = new SparkMax(12, MotorType.kBrushless);
            climbMotor.getEncoder().setPosition(0.0);
      
            pid = new PIDController(0.0025+0.0023889, 0, 0);
            pid.enableContinuousInput(-180.0, 180.0);
        
         //   liftMotor = new SparkMax(12, MotorType.kBrushed);
        } 
        else {
            // old drive base CAN IDs
            leftFront = new SwerveModule(0,348.0-90, 12,17, zeroMode,oldDriveBase);
            rightFront = new SwerveModule(1,70.1-270, 20,2, zeroMode,oldDriveBase);
            rightBack = new SwerveModule(2,2.31-180, 14,32, zeroMode,oldDriveBase);
            leftBack = new SwerveModule(3,69.3-180, 29,15, zeroMode,oldDriveBase);
            pid = new PIDController(0.0523889, 0, 0);
            pid.enableContinuousInput(-180.0, 180.0);
        }

        Translation2d frontRight = new Translation2d(0.33655, -0.33655); 
        Translation2d frontLeft = new Translation2d(0.33655, 0.33655); 
        Translation2d backRight = new Translation2d(-0.33655, -0.33655); 
        Translation2d backLeft = new Translation2d(-0.33655, 0.33655);     
  
        kinematics = new SwerveDriveKinematics(frontRight, frontLeft, backRight, backLeft);

        odometry = new SwerveDrivePoseEstimator (kinematics, gyro.getRotation2d(), 
            new SwerveModulePosition[]{
                rightFront.getPosition(),
                leftFront.getPosition(),
                rightBack.getPosition(),
                leftBack.getPosition(),    
            },Pose2d.kZero,
            stateStdDevs,
            visionMeasurementStdDevs);
    }
        /**
         * Field Relative Drive.
         */
      void drive(double xSpeed, double ySpeed, double rot, boolean highSpeed, boolean lowSpeed) {
        ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(control);
    
        moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
        moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
        moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
        moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));
    
        rightFront.turny(moduleStates[0].angle.getDegrees());
        leftFront.turny(moduleStates[1].angle.getDegrees());
        rightBack.turny(moduleStates[2].angle.getDegrees());
        leftBack.turny(moduleStates[3].angle.getDegrees());
      
        if (lowSpeed) {
            rightFront.movey(moduleStates[0].speedMetersPerSecond*0.25);
            leftFront.movey(moduleStates[1].speedMetersPerSecond*0.25);
            rightBack.movey(moduleStates[2].speedMetersPerSecond*0.25);
            leftBack.movey(moduleStates[3].speedMetersPerSecond*0.25);
        
        } else {
            rightFront.movey(moduleStates[0].speedMetersPerSecond*0.87);
            leftFront.movey(moduleStates[1].speedMetersPerSecond*0.87);
            rightBack.movey(moduleStates[2].speedMetersPerSecond*0.87);
            leftBack.movey(moduleStates[3].speedMetersPerSecond*0.87);
        }          
        dashboard.updateDashboardSwerveModules(leftFront,rightFront,leftBack,rightBack);
        updateOdometry();
    }

    double kMaxSpeed = 3.0;//
    void driveLL(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        ChassisSpeeds control = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());

        if(!fieldRelative) {
            control = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        control = ChassisSpeeds.discretize(control, periodSeconds);

        var moduleStates = kinematics.toSwerveModuleStates(control);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeed);

        moduleStates[0].optimize(Rotation2d.fromDegrees(rightFront.getRotation()));
        moduleStates[1].optimize(Rotation2d.fromDegrees(leftFront.getRotation()));
        moduleStates[2].optimize(Rotation2d.fromDegrees(rightBack.getRotation()));
        moduleStates[3].optimize(Rotation2d.fromDegrees(leftBack.getRotation()));

        rightFront.turny(moduleStates[0].angle.getDegrees());
        leftFront.turny(moduleStates[1].angle.getDegrees());
        rightBack.turny(moduleStates[2].angle.getDegrees());
        leftBack.turny(moduleStates[3].angle.getDegrees());

        rightFront.movey(moduleStates[0].speedMetersPerSecond*0.25);
        leftFront.movey(moduleStates[1].speedMetersPerSecond*0.25);
        rightBack.movey(moduleStates[2].speedMetersPerSecond*0.25);
        leftBack.movey(moduleStates[3].speedMetersPerSecond*0.25);
    }

    void resetGyro() {
        gyro.reset();
    }
    double getGyro() {
        return (gyro.getAngle() % 360);
    }
    void gyroOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }

    void updaterobotorientation1() {
        LimelightHelpers.SetRobotOrientation("limelight", odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2;
             
        //Blue
        if(alliancecolor == Alliance.Blue) {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
        }
        //Red
        else {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        }

        if(mt2 != null && mt2.tagCount > 0)
        {
            odometry.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);  
        }
    }
    public void updateOdometry() {
        updaterobotorientation1();

        var pose = odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                rightFront.getPosition(),
                leftFront.getPosition(),
                rightBack.getPosition(),
                leftBack.getPosition()
        }); 

        dashboard.updatePose(
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        );
      }

      RawFiducial GetAprilTagTelemotry(int aprilTag) {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
            for (RawFiducial fiducial : fiducials) {
                    int id = fiducial.id;
                    double txnc = fiducial.txnc;
                    double tync = fiducial.tync;
                    double ta = fiducial.ta;
                    double distToCamera = fiducial.distToCamera;
                    double distToRobot = fiducial.distToRobot;
                    double ambiguity = fiducial.ambiguity; 

                    if(id == aprilTag) {
                        return fiducial;
                    }
                    if(txnc == aprilTag){
                        return fiducial;
                    }
                    if(tync == aprilTag){
                        return fiducial;
                    }
                    if(ta == aprilTag){
                        return fiducial;
                    }
                //System.out.println("Tag: " + id);
            }
            return null;
    }

    // void liftLevelSet(int level) {
    //     currentHeightLift = 2.66666666667*liftMotor.getEncoder().getPosition();
    //     switch(level) {
    //         case 1: 
    //         targetLiftHeight = 0;
    //         break;

    //         case 2:
    //         targetLiftHeight = 0;
    //         break;

    //         case 3:
    //         targetLiftHeight = 0;
    //         break;

    //         case 4:
    //         targetLiftHeight = 431.8;
    //         break;

    //     }
    //     pidLift = new PIDController(.00001,0 ,0 );
    //     liftSpeed = pidLift.calculate(currentHeightLift, targetLiftHeight);
    //     liftMotor.set(liftSpeed);
    // }

    void outTakeSet(double speed) {
        //System.out.println("outTakeSet: " + speed);
        outTakeMotorOuter.set(speed);
        outTakeMotorInner.set(-speed);
    }
    void turnBotToAngle(double targetAngle){
        double currentAngle = gyro.getAngle();
        double rotationVal = pid.calculate(currentAngle,targetAngle);
        rotationVal = MathUtil.clamp(rotationVal,-0.3, 0.3);
        drive(0, 0, rotationVal, false, false);
    }
    void armSet(int targetAngle) {
        //currentAngleArm = armMotor.getEncoder().getPosition();
        pidArm = new PIDController(.1023, 0, 0);
        armSpeed = pidArm.calculate(currentAngleArm, targetAngle*125);
        armMotor.set(armSpeed);
    }

    public AHRS getEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoder'");
    }

    enum ShortcutType {
        PLAYER, L1, L2, L3, L4
    }

    double armPlayerPosition = 0;
    double armL1Position = -10;
    double armL2Position = 0;
    double armL3Position = -100;
    double armL4Position = -92.8;

    double liftPlayerHeight = 34;
    double liftL1Height = 34.3;
    double liftL2Height = 60.8;
    double liftL3Height = 11.5;
    double liftL4Height = 66.4;

    double armClampSpeed = 0.3;
    double liftClampSpeed = 0.7;

    double armMotorSpeed = 0;
    double liftMotorSpeed = 0.0;

    double liftMotorPosition = 0.0;
    double armPosition = 0.0;

    double ShortCutLift(ShortcutType shortcut) {
        liftMotorSpeed = 0.0;

        if (armMotor == null || liftMotor == null) {
            System.out.println("Lift/Arm Motor Issue--Abort---");
            return 0;
        }

        liftMotorPosition = potLift.get();

        //Human Player
        if(shortcut == ShortcutType.PLAYER) {
            liftMotorSpeed = pidShortcutLift.calculate(liftMotorPosition, liftPlayerHeight);
        }

        //L1
        if(shortcut == ShortcutType.L1) {
            liftMotorSpeed = pidShortcutLift.calculate(liftMotorPosition, liftL1Height);
        }

        //L2
        if(shortcut == ShortcutType.L2) {
            liftMotorSpeed = pidShortcutLift.calculate(liftMotorPosition, liftL2Height);
        }
        
        //L3
        if(shortcut == ShortcutType.L3) {
            liftMotorSpeed = pidShortcutLift.calculate(liftMotorPosition, liftL3Height);
        }

        //L4 
        if(shortcut == ShortcutType.L4) {
            liftMotorSpeed = pidShortcutLift.calculate(liftMotorPosition, liftL4Height);
        }
            
        if (!limitSensorBottom.get() && liftMotorSpeed > 0) {
            liftMotorSpeed = 0;
            liftMotor.getEncoder().setPosition(0.0);
        }

        liftMotorSpeed = MathUtil.clamp(liftMotorSpeed, -liftClampSpeed, liftClampSpeed);

        return liftMotorSpeed;
    }

    double ShortCutArm(ShortcutType shortcut) {
        armMotorSpeed = 0;

        if (armMotor == null || liftMotor == null) {
            System.out.println("Lift/Arm Motor Issue--Abort---");
            return 0;
        }

        armPosition = armMotor.getPosition().getValueAsDouble();

        //Human Player
        if(shortcut == ShortcutType.PLAYER) {
            armMotorSpeed = pidShortcutArm.calculate(armPosition, armPlayerPosition);
        }

        //L1
        if(shortcut == ShortcutType.L1) {
            armMotorSpeed = pidShortcutArm.calculate(armPosition, armL1Position);
        }

        //L2
        if(shortcut == ShortcutType.L2) {
            armMotorSpeed = pidShortcutArm.calculate(armPosition, armL2Position);
        }
        
        //L3
        if(shortcut == ShortcutType.L3) {
            armMotorSpeed = pidShortcutArm.calculate(armPosition, armL3Position);
        }

        //L4 
        if(shortcut == ShortcutType.L4) {
            armMotorSpeed = pidShortcutArm.calculate(armPosition, armL4Position);
        }
            
        armMotorSpeed = MathUtil.clamp(armMotorSpeed, -armClampSpeed, armClampSpeed);

        System.out.println("armPosition: " + armPosition + " liftMotorPosition: " + liftMotorPosition + " liftMotorSpeed: " + liftMotorSpeed + " armMotorSpeed: " + armMotorSpeed);

        return armMotorSpeed;
    }


    double LLGetY() {
        return -1;
    }
}
