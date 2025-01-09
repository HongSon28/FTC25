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

    /* Alliance specific Poses
       0 = Blue, 1 = Red
     */
    public static final Pose2d[] STARTING_POSE  = { new Pose2d(0,0,0),
                                                    new Pose2d(0,0,0)};
    public static final Pose2d[] OBSERVATION_ZONE = { new Pose2d(0,0,0),
                                                      new Pose2d(0,0,0)};
}
