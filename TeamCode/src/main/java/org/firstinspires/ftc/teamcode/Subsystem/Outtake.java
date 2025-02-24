package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig;

public class Outtake {
    private DcMotor slideMotor;
    public ServoImplEx servoArm, servoWrist, servoClaw;
    private static final int MAX_POS = 5750, MIN_POS = 0, FLIP_POS = 3500, INTAKE_POS = 1500; // 5750
    private int slideTarget = MIN_POS;
    private static final double DEFAULT_SPEED = 1.0;
    public static final double[] ARM_POS = {0.55 /* tune */, 0.3, 0};
    private final double ARM_SPEED = 0.0075;
    private static final double[] WRIST_POS = {0.33, 0.22, 0.28, 0};
    private static final double CLAW_POS = 0.8;
    private boolean clawState, wristState;
    public int armState, lastArm;

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry) {
        clawState = false;
        armState = lastArm = 0;

        slideMotor = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_OUTTAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setPower(DEFAULT_SPEED);
        slideMotor.setTargetPosition(MIN_POS);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoArm = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_ARM);
        servoArm.setPwmRange(RobotConfig.DS3225_PWM_RANGE);
        servoArm.setPosition(ARM_POS[armState]);

        servoWrist = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_WRIST);
        servoWrist.setPwmRange(RobotConfig.DS3225_PWM_RANGE);
        servoWrist.setPosition(WRIST_POS[0]);

        servoClaw = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_CLAW);
        servoClaw.setPwmRange(RobotConfig.MG996R_PWM_RANGE);
        servoClaw.setPosition(0);
        clawState = true;
    }

    public void setWrist(boolean state) {
        wristState = state;
    }

    public void setSlide(boolean state) {

        if (state) slideMotor.setTargetPosition(MAX_POS);
        else slideMotor.setTargetPosition(MIN_POS);
    }

    public void setArm(int delta) {
        if (delta != lastArm)
            armState += delta;
        lastArm = delta;
        if (armState < 0) armState = 0;
        if (armState > 2) armState = 2;
    }

    public void setClaw(boolean state) {
        clawState = state;
    }

    public void control(Telemetry telemetry) {

        double curPos = servoArm.getPosition();
        double targetPos = ARM_POS[armState];
        telemetry.addData("curPos", curPos);
        telemetry.addData("targetPos", targetPos);
        if (curPos < targetPos) {
            if (targetPos - curPos <= ARM_SPEED) curPos = targetPos;
            else curPos += ARM_SPEED;
        } else {
            if (curPos - targetPos <= ARM_SPEED) curPos = targetPos;
            else curPos -= ARM_SPEED;
        }
        servoArm.setPosition(curPos);

        telemetry.addData("State", armState);
        telemetry.addData("Current Arm Pos:",ARM_POS[armState]);
        telemetry.addData("Target Pos",slideMotor.getTargetPosition());
        telemetry.addData("Current Slide Pos",slideMotor.getCurrentPosition());

        if (wristState) servoWrist.setPosition(WRIST_POS[3]);
        else servoWrist.setPosition(WRIST_POS[armState]);

        telemetry.addData("Wrist Position:", servoWrist.getPosition());

        if (!clawState) servoClaw.setPosition(CLAW_POS);
        else servoClaw.setPosition(0);

        telemetry.update();
    }
}
