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
    private DcMotor motorSlide;
    private CRServo servo;
    public ServoImplEx servoAngle;
    private NormalizedColorSensor colorSensor;
    private static final double SPEED = 1;
    private static final int MAX_POS = 1550;
    private static final double REV_SPEED = -1;
    private static final double DOWN_POS = 1.0;
    private static final double SLIDE_SPEED = 0.75;
    private int allianceID; // 0 = Blue, 1 = Red;
    public NormalizedRGBA colors;
    public float[] hsvValues = new float[3];
    public double dist;
    public boolean extended;

    public Intake(HardwareMap hardwareMap,int alliance) {
        servo = hardwareMap.get(CRServo.class, RobotConfig.SERVO_INTAKE);
        servo.setPower(0);

        motorSlide = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_INTAKE_SLIDE);
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setPower(SLIDE_SPEED);
        motorSlide.setTargetPosition(0);
        motorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extended = false;

        servoAngle = hardwareMap.get(ServoImplEx.class, RobotConfig.SERVO_ANGLE);
        servoAngle.setPwmRange(RobotConfig.GOBILDA_PWM_RANGE);
        servoAngle.setPosition(DOWN_POS);

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
    public void setServo(int state) {
        if (state == 2) servo.setPower(SPEED);
        else if (state == 1) servo.setPower(REV_SPEED);
        else servo.setPower(0);
    }

    public void setAngle(boolean state) {
        if (state) servoAngle.setPosition(0.09);
        else servoAngle.setPosition(DOWN_POS);
    }

    public void setSlide(boolean state) {
        if (state) {
            motorSlide.setTargetPosition(MAX_POS);
        }
        else
        {
            motorSlide.setTargetPosition(0);
        }
        extended = state;
    }
}
