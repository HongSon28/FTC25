package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig;

public class Outtake {
    private DcMotor slideMotor;
    public ServoImplEx servoArm, servoWrist, servoClaw;
    private static final int MAX_POSI = 5000;
    private static final double DEFAULT_SPEED = 0.75;
    public static final double[] ARM_POS = {1, 0.66, 0.33, 0};
    private final double ARM_SPEED = 0.004;
    private static final double[] WRIST_POS = {0, 0.25, 0.66};
    private static final double CLAW_POS = 0.8;
    private boolean slideState, clawState;
    private int wristState, lastWrist;
    private int armState, lastArm;
    public Outtake(HardwareMap hardwareMap, Telemetry telemetry) {
        slideState = clawState = false;
        wristState = lastWrist = 0;
        armState = lastArm = 0;

        slideMotor = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_OUTTAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setPower(DEFAULT_SPEED);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoArm = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_ARM);
        servoArm.setPwmRange(RobotConfig.DS3225_PWM_RANGE);
        servoArm.setPosition(1);

        servoWrist = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_WRIST);
        servoWrist.setPwmRange(RobotConfig.GOBILDA_PWM_RANGE);
        servoWrist.setPosition(WRIST_POS[wristState]);

        servoClaw = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_CLAW);
        servoClaw.setPwmRange(RobotConfig.MG996R_PWM_RANGE);


    }

    public void setSlide(boolean state) {
        slideState = state;
    }

    public void setArm(int delta) {
        if (delta != lastArm)
            armState += delta;
        lastArm = delta;
        if (armState < 0) armState = 0;
        if (armState > 3) armState = 3;
    }

    public void setWrist(int state) {
        if (state != lastWrist)
            wristState += state;
        lastWrist = state;
        if (wristState < 0) wristState = 0;
        if (wristState > 2) wristState = 2;
    }

    public void setClaw(boolean state) {
        clawState = state;
    }

    public void control(Telemetry telemetry) {
        if (slideState) slideMotor.setTargetPosition(MAX_POSI);
        else slideMotor.setTargetPosition(0);

        double curPos = servoArm.getPosition();
        double targetPos = ARM_POS[armState];
        if (curPos < targetPos) {
            if (targetPos - curPos <= ARM_SPEED) curPos = targetPos;
            else curPos += ARM_SPEED;
        } else {
            if (curPos - targetPos <= ARM_SPEED) curPos = targetPos;
            else curPos -= ARM_SPEED;
        }
        servoArm.setPosition(curPos);
        telemetry.addData("State", armState);
        telemetry.update();

        servoWrist.setPosition(WRIST_POS[wristState]);

        if (clawState) servoClaw.setPosition(CLAW_POS);
        else servoClaw.setPosition(0);
    }
}
