package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class Intake {
    private CRServo servo;
    private NormalizedColorSensor sensor;
    private final double SPEED = 0.75;
    public Intake(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, RobotConfig.SERVO_INTAKE);
        sensor = hardwareMap.get(NormalizedColorSensor.class, RobotConfig.SENSOR);
        servo.setPower(0);
    }

    public NormalizedRGBA get() {
        return sensor.getNormalizedColors();
    }
    public void setServo(boolean state) {
        if (state) servo.setPower(SPEED);
        else servo.setPower(0);
    }
}
