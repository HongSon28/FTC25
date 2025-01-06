package org.firstinspires.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;

public class LinearSlide {
    private DcMotor slideMotor;
    private static final int MAX_POSI = -5250; //lol giá trị thí nghiệm lấy từ năm ngoái
    private static final double SPEED_CODINH = 0.4; //lol giá trị thí nghiệm lấy từ năm ngoái
    private static final double NUA_VOI_RATIO = 0.5; //lol giá trị thí nghiệm lấy từ năm ngoái
    private double motorSpeed;

    public LinearSlide(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, RobotConfig.DC_SLIDE);
        motorSpeed = SPEED_CODINH;

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

            case 1: // Partially extended position
                slideMotor.setTargetPosition((int) (MAX_POSI*NUA_VOI_RATIO));
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
}
