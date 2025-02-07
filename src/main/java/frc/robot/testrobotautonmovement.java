package frc.robot;

public class testrobotautonmovement {
    public class RobotMovement {

        private Motor leftMotor;
        private Motor rightMotor;
    
        public RobotMovement(Motor leftMotor, Motor rightMotor) {
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
        }
    
        public void moveForward(int speed, int duration) throws InterruptedException {
           
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            Thread.sleep(duration);
            stop();
        }
    
        public void turnRight(int speed, int duration) throws InterruptedException {
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(-speed); // Reverse right motor for turning
            Thread.sleep(duration);
            stop();
        }
    
        public void stop() {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
        }
    
        public static void main(String[] args) throws InterruptedException {
            // Example Usage: Replace with your actual motor implementations
            Motor leftMotor = new Motor("LeftMotor");
            Motor rightMotor = new Motor("RightMotor");
            //RobotMovement robot = new RobotMovement(leftMotor, rightMotor);
    
            //robot.moveForward(50, 2000); // Move forward at speed 50 for 2000ms
            //robot.turnRight(50, 1000); // Turn right at speed 50 for 1000ms
        }
    
        //Dummy class representing a motor
        static class Motor {
            private String name;
    
            public Motor(String name) {
                this.name = name;
            }
    
            public void setSpeed(int speed) {
                System.out.println(name + " speed set to " + speed);
            }
        }
    }
}
