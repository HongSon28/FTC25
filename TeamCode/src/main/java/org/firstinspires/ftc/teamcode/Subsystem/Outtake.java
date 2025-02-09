package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Outtake {
    private DcMotor slideMotor;
    private ServoImplEx servo;
    private static final int MAX_POSI = -5250;
    private static final double DEFAULT_SPEED = 0.75;
    private static final double SERVO_POS = 0.75;
    public Outtake(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_OUTTAKE);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(-DEFAULT_SPEED);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_OUTTAKE);
        servo.setPwmRange(RobotConfig.GOBILDA_PWM_RANGE);
        servo.setPosition(0);
    }

    public void setSlide(boolean state) {
        if (state) slideMotor.setTargetPosition(MAX_POSI);
        else slideMotor.setTargetPosition(0);
    }

    public void setServo(boolean state) {
        if (!state) servo.setPosition(0);
        else {
            servo.setPosition(SERVO_POS);
            //if (slideMotor.getCurrentPosition() == MAX_POSI) servo.setPosition(SERVO_POS);
            //else servo.setPosition(0);
        }
    }
}
