package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Calibration Tool")
public class SlideCalibration extends LinearOpMode {
    private DcMotor linearSlide;

    @Override
    public void runOpMode() {
        linearSlide = hardwareMap.get(DcMotor.class, RobotConfig.MOTOR_OUTTAKE);

        // Brake function: prevent falling due to gravity
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready for calibration");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double power = -gamepad1.left_stick_y * 1.0; // Reduced sensitivity
            linearSlide.setPower(power);

            telemetry.addData("Slide Position", linearSlide.getCurrentPosition());
            telemetry.addData("Current Power", power);
            telemetry.update();
        }

        linearSlide.setPower(0);
    }
}
