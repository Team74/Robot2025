package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.testrobotautonmovement.RobotMovement.Motor;

public class StartToReef {
    SwerveModule rightFront;
    SwerveModule rightBack;
    SwerveModule leftFront;
    SwerveModule leftBack;
    SparkMax liftMotor;
    String state = "start";
    Servo outtakeServo;

    public StartToReef(SwerveModule[] list , SparkMax lift, Servo servo) {
        rightFront = list[0];
        rightBack = list[1];
        leftFront = list[2];
        leftBack = list[3];
       liftMotor = lift;
       outtakeServo = servo;
       state = "start";
    }

    void RunS2R(int time) {
        switch (state) {
            // first number in if statement is time in seconds
            case "start":
            MotorSet(0.0,0.0,false);
            if (time > 0*50) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            MotorSet(0.1,0.0,false);
            if (time > 3*50) {
                time = 0;
                state = "lift";
                break;
            }

            case "lift":
            MotorSet(0.0,0,false);
            liftSet(4);
            if (time > 1*50) {
                time = 0;
                state = "dumpy";
                break;
            }

            case "dumpy":
            MotorSet(0.0,0,false);
            liftSet(1);
            outtakeServo.set(1);
            if (time > .5*50) {
                time = 0;
                state = "down";
                break;
            }

            case "down":
            MotorSet(0.0,0,false);
            liftSet(0);
            outtakeServo.set(0);
            if (time > 1*50) {
                time = 0;
                state = "stop";
                break;
            }

            case "stop":
            MotorSet(0.0,0,false);
                state = "stop";
                break;
            

        }
    }
    void oldRunS2R(int time) {
        System.out.println(state);
        switch (state) {
            // first number in if statement is time in seconds
            case "start":
            MotorSet(0.0,0.0,false);
            if (time > 0) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            MotorSet(0.2,0.0,false);
            if (time > 3*50) {
                time = 0;
                state = "lift";
                break;
            }

            case "lift":
            MotorSet(0.0,0,false);
           
            if (time > 1*50) {
                time = 0;
                state = "dumpy";
                break;
            }

            case "dumpy":
            MotorSet(0.0,0,false);
           
          
            if (time > .5*50) {
                time = 0;
                state = "down";
                break;
            }

            case "down":
            MotorSet(0.0,0,false);
           
           
            if (time > 1*50) {
                time = 0;
                state = "stop";
                break;
            }

            case "stop":
            MotorSet(0.0,0,false);
                state = "stop";
                break;
            

        }
    }


    void MotorSet(double speed, double angle, boolean turning){
        if(turning) {
        rightFront.turny(45);
        leftFront.turny( 135);
        rightBack.turny(135);
        leftBack.turny(45);
        rightFront.movey(speed);
        leftFront.movey(speed);
        rightBack.movey(-speed);
        leftBack.movey(-speed);
        } else
        rightFront.turny(angle);
        leftFront.turny(angle);
        rightBack.turny(angle);
        leftBack.turny(angle);
        rightFront.movey(speed);
        leftFront.movey(speed);
        rightBack.movey(speed);
        leftBack.movey(speed);
    }

    void liftSet(int level) {
        if (level == 1) {
            liftMotor.set(0.0);
        } else
        if (level == 2) {
            liftMotor.set(0.3333);
        } else
        if (level == 3) {
            liftMotor.set(0.6666);
        } else
        if (level == 4) {
            liftMotor.set(.1);
        } else
        if (level == 0) {
            liftMotor.set(-.1);
        } 
    }
}
