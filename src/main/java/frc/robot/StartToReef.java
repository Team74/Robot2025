package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.testrobotautonmovement.RobotMovement.Motor;

public class StartToReef {

    SparkMax liftMotor;
    String state = "start";
    Servo outtakeServo;
    int time;
    driveTrain driveTrain;

    public StartToReef(SparkMax lift, Servo servo, driveTrain driveTrain1) {
        time = 0;
        liftMotor = lift;
        outtakeServo = servo;
        driveTrain = driveTrain1;
        System.out.println(state);
    }

    void RunS2R() {
        time++;
        switch (state) {
            // first number in if statement is time in seconds
            case "start":
            driveTrain.drive(0, 0, 0, false);
            if (time > 0*50) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            driveTrain.drive(0.2, 0, 0, false);   
            if (time > 3*50) {
                time = 0;
                state = "lift";
                break;
            }

            case "lift":
            driveTrain.drive(0, 0, 0, false);
            liftSet(4);
            if (time > 1*50) {
                time = 0;
                state = "dumpy";
                break;
            }

            case "dumpy":
            driveTrain.drive(0, 0, 0, false);
            liftSet(4);
            outtakeServo.set(1);
            if (time > .5*50) {
                time = 0;
                state = "down";
                break;
            }

            case "down":
            driveTrain.drive(0, 0, 0, false); 
            liftSet(0);
            outtakeServo.set(0);
            if (time > 1*50) {
                time = 0;
                state = "stop";
                break;
            }

            case "stop":
            driveTrain.drive(0, 0, 0, false);
                state = "stop";
                break;
            

        }
    }

    void oldRunS2R() {
        System.out.println(state + time);
        time++;
        switch (state) {
    
            case "start":
            driveTrain.drive(0, 0, 0, false);
            if (time > 0) {
                time = 0;
                state = "move1";
                break;
            }

            case "move1":
            driveTrain.drive(0.2, 0, 0, false);
            if (time > 150) {
                time = 0;
                state = "lift";
                break;
            }

            case "lift":
            driveTrain.drive(0, 0, 0, false);
            if (time > 50) {
                time = 0;
                state = "dumpy";
                break;
            }

            case "dumpy":
            driveTrain.drive(0, 0, 0, false);
            if (time > 25) {
                time = 0;
                state = "down";
                break;
            }

            case "down":
            driveTrain.drive(0, 0, 0, false);
            if (time > 50) {
                time = 0;
                state = "stop";
                break;
            }

            case "stop":
            driveTrain.drive(0, 0, 0, false);                
            state = "stop";
                break; 
            

        }
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
