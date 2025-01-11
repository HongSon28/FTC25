package org.firstinspires.ftc.teamcode.Subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {
    private CRServo servo;
    private NormalizedColorSensor colorSensor;
    private final double SPEED = 0.75;
    private final double REV_SPEED = -0.25;
    private int allianceID; // 0 = Blue, 1 = Red;
    private float[] hsvValues = new float[3];

    public Intake(HardwareMap hardwareMap,int alliance) {
        servo = hardwareMap.get(CRServo.class, RobotConfig.SERVO_INTAKE);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.SENSOR);
        servo.setPower(0);
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
    public void setServo(int state) {
        if (!check()) {
            servo.setPower(0);
            return;
        }
        if (state == 2) servo.setPower(SPEED);
        else if (state == 1) servo.setPower(REV_SPEED);
        else servo.setPower(0);
    }
}
