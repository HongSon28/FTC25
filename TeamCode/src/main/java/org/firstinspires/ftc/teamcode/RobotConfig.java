package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotConfig {
    /* drivetrain motor names */
    public static final String DRIVE_LEFT_BACK = "driveLeftBack";
    public static final String DRIVE_RIGHT_BACK = "driveRightBack";
    public static final String DRIVE_LEFT_FRONT = "driveLeftFront";
    public static final String DRIVE_RIGHT_FRONT = "driveRightFront";

    /* Motor names */
    public static final String MOTOR_SLIDE = "motorSlide";
    public static final String MOTOR_OUTTAKE = "motorOuttake";

    /* Servo names */
    public static final String SERVO_INTAKE = "servoIntake";
    public static final String SERVO_OUTTAKE = "servoOuttake";

    /* Sensor names */
    public static final String SENSOR = "colorSensor";

    public static final Pose2d[] STARTING_POSE  = { new Pose2d(120,0,Math.toRadians(-90)),
                                                    new Pose2d(72,0,Math.toRadians(-90))};
    /*
        0 = Closer to Net Zone
        1 = Closer to Observation Zone
    */
    public static final Pose2d NET_ZONE = new Pose2d(120,12, Math.toRadians(45));
}
