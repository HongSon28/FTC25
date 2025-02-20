package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.PwmControl;

public class RobotConfig {
    /* drivetrain motor names */
    public static final String DRIVE_LEFT_BACK = "driveLeftBack";
    public static final String DRIVE_RIGHT_BACK = "driveRightBack";
    public static final String DRIVE_LEFT_FRONT = "driveLeftFront";
    public static final String DRIVE_RIGHT_FRONT = "driveRightFront";

    /* Motor names */
    public static final String MOTOR_SLIDE = "motorSlide";
    public static final String MOTOR_INTAKE = "motorIntake";
    public static final String MOTOR_OUTTAKE = "motorOuttake";

    /* Servo names */
    public static final String SERVO_ARM = "servoArm";
    public static final String SERVO_WRIST = "servoWrist";
    public static final String SERVO_CLAW = "servoClaw";
    public static final String SERVO_LOCK = "servoLock";
    public static final String SERVO_SLIDE_LEFT = "servoSlideLeft";
    public static final String SERVO_SLIDE_RIGHT = "servoSlideRight";
    public static final String SERVO_LIFT_LEFT = "servoLiftLeft";
    public static final String SERVO_LIFT_RIGHT = "servoLiftRight";

    /* Servo PWM Range */
    public static final PwmControl.PwmRange GOBILDA_PWM_RANGE = new PwmControl.PwmRange(500,2500);
    public static final PwmControl.PwmRange DS3225_PWM_RANGE = new PwmControl.PwmRange(500,2500);

    public static final PwmControl.PwmRange SG90_PWM_RANGE = new PwmControl.PwmRange(1000,2000);
    public static final PwmControl.PwmRange MG996R_PWM_RANGE = new PwmControl.PwmRange(1000,2000);

    /* Sensor names */
    public static final String SENSOR = "colorSensor";


    /* Color threshold */
    public static final int RED_THRESHOLD = 30; //Hue
    public static final int BLUE_THRESHOLD = 180; //Hue

    /* Poses */
    public static final Pose2d[] STARTING_POSE  = { new Pose2d(120,0,Math.toRadians(-90)),
                                                    new Pose2d(72,0,Math.toRadians(-90))};
    /*
        0 = Closer to Net Zone
        1 = Closer to Observation Zone
    */
    public static final Pose2d NET_ZONE = new Pose2d(120,12, Math.toRadians(45));
}
