package org.firstinspires.ftc.teamcode.Subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {
    private DcMotor motor;
    private ServoImplEx servoLock, servoLiftLeft, servoLiftRight;
    private CRServo servoSlideLeft, servoSlideRight;
    private NormalizedColorSensor colorSensor;
    private final double SPEED = 1;
    private final double REV_SPEED = -1;
    private final double SLIDE_SPEED = 0.75;
    private final double OPEN_ANGLE = 0;
    private final double LOCK_ANGLE = 1;
    private int allianceID; // 0 = Blue, 1 = Red;
    public NormalizedRGBA colors;
    public float[] hsvValues = new float[3];
    public double dist;
    public double curPos = 0;

    public Intake(HardwareMap hardwareMap,int alliance) {
        motor = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_INTAKE);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.SENSOR);
        motor.setPower(0);

        servoLock = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_LOCK);
        servoLock.setPwmRange(RobotConfig.SG90_PWM_RANGE);
        servoLock.setPosition(LOCK_ANGLE);

        servoSlideLeft = hardwareMap.get(CRServo.class, RobotConfig.SERVO_SLIDE_LEFT);
        servoSlideRight = hardwareMap.get(CRServo.class, RobotConfig.SERVO_SLIDE_RIGHT);
        servoSlideLeft.setPower(0);
        servoSlideRight.setPower(0);

        servoLiftLeft = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_LIFT_LEFT);
        servoLiftLeft.setPwmRange(RobotConfig.GOBILDA_PWM_RANGE);
        servoLiftLeft.setPosition(0);

        servoLiftRight = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_LIFT_RIGHT);
        servoLiftRight.setPwmRange(RobotConfig.GOBILDA_PWM_RANGE);
        servoLiftRight.setPosition(0);

        this.allianceID = alliance;
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public int color() { //0 = blue, 1 = red, 2 = neutral
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] <= RobotConfig.RED_THRESHOLD) return 1;
        if (hsvValues[0] >= RobotConfig.BLUE_THRESHOLD) return 0;
        return 2;
    }

    public void get(Telemetry telemetry) {
        colors = colorSensor.getNormalizedColors();
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.update();
    }

    public boolean check() {
        dist = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
        if (dist > 20) return true;
        int color = color();
        if (color == 2) return true;
        return (color == allianceID);
    }
    public void setMotor(int state) {
        if (state == 2) motor.setPower(SPEED);
        else if (state == 1) motor.setPower(REV_SPEED);
        else motor.setPower(0);
    }

    public void setLift(int state) {
        double pos = 0;
        if (state == 1) pos = 0.5;
        else if (state == 2) pos = -0.5;
        servoLiftLeft.setPosition(pos);
        servoLiftRight.setPosition(-pos);
    }

    public void setSlide(double power) {
        servoSlideLeft.setPower(-power);
        servoSlideRight.setPower(power);
    }

    public void setLock(boolean state) {
        if (!state) servoLock.setPosition(OPEN_ANGLE);
        else servoLock.setPosition(LOCK_ANGLE);
    }
}
