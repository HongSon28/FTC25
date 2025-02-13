package org.firstinspires.ftc.teamcode.Subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {
    private DcMotor motor;
    private ServoImplEx servoLock;
    private ServoImplEx servoSlideLeft, servoSlideRight;
    private NormalizedColorSensor colorSensor;
    private final double SPEED = 1;
    private final double REV_SPEED = -1;
    private final double SLIDE_SPEED = 0.75;
    private final double OPEN_ANGLE = 1;
    private final double LOCK_ANGLE = 0.5;
    private int allianceID; // 0 = Blue, 1 = Red;
    private float[] hsvValues = new float[3];
    private double curPos = 0;

    public Intake(HardwareMap hardwareMap,int alliance) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_INTAKE);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.SENSOR);
        motor.setPower(0);

        servoLock = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_LOCK);
        servoLock.setPwmRange(RobotConfig.SG90_PWM_RANGE);
        servoLock.setPosition(LOCK_ANGLE);

        servoSlideLeft = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_SLIDE_LEFT);
        servoSlideRight = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_SLIDE_RIGHT);
        servoSlideLeft.setPwmRange(RobotConfig.MG996R_PWM_RANGE);
        servoSlideRight.setPwmRange(RobotConfig.MG996R_PWM_RANGE);
        servoSlideLeft.setPosition(curPos);
        servoSlideRight.setPosition(curPos);

        this.allianceID = alliance;
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public int color() { //0 = blue, 1 = red, 2 = neutral
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] <= RobotConfig.RED_THRESHOLD) return 1;
        if (hsvValues[0] >= RobotConfig.BLUE_THRESHOLD) return 0;
        return 2;
    }
    public boolean check() {
        int color = color();
        if (color == 2) return true;
        return (color == allianceID);
    }
    public void setMotor(int state) {
        /*
        if (!check()) {
            motor.setPower(0);
            return;
        }*/
        if (state == 2) motor.setPower(SPEED);
        else if (state == 1) motor.setPower(REV_SPEED);
        else motor.setPower(0);
    }

    public void setSlide(double delta) {
        delta *= 0.01;
        curPos += delta;
        if (curPos < -1.0) curPos = -1.0;
        if (curPos > 1.0) curPos = 1.0;
        servoSlideLeft.setPosition(curPos);
        servoSlideRight.setPosition(-curPos);
    }

    public void setLock(boolean state) {
        if (!state) servoLock.setPosition(OPEN_ANGLE);
        else servoLock.setPosition(LOCK_ANGLE);
    }
}
