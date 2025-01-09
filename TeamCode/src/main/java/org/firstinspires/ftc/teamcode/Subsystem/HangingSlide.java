package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class HangingSlide {
    private DcMotor slideMotor;
    private static final int MAX_POSI = -5250; //lol giá trị thí nghiệm lấy từ năm ngoái
    private static final double DEFAULT_SPEED = 0.4; //lol giá trị thí nghiệm lấy từ năm ngoái
    private static final double RATIO = 0.5; //lol giá trị thí nghiệm lấy từ năm ngoái
    private double motorSpeed;

    public HangingSlide(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, RobotConfig.MOTOR_SLIDE);
        motorSpeed = DEFAULT_SPEED;

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(-motorSpeed);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveToState(int targetState) {
        switch (targetState) {
            case 2: // kéo dãn full
                slideMotor.setTargetPosition(MAX_POSI);
                break;

            case 1: // dãn theo điều chỉnh ratio
                slideMotor.setTargetPosition((int) (MAX_POSI*RATIO));
                break;

            default: // kéo thu về
                slideMotor.setTargetPosition(0);
                break;
        }
    }

    public void boostPower() {
        motorSpeed = 1.0; //max motor speed nếu nhanh quá có thể chỉnh
        slideMotor.setPower(-motorSpeed);
    }

    public void resetPower() {
        motorSpeed = DEFAULT_SPEED;
        slideMotor.setPower(-motorSpeed);
    }
}
