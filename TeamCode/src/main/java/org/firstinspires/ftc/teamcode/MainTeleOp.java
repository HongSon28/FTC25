package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private PinpointDrive drivetrain;
    private Intake intake;

    @Override
    public void runOpMode() {
        drivetrain = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));
        intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double x = -gamepad1.left_stick_x;

            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            x
                    ),
                    -gamepad1.right_stick_x
            ));
            drivetrain.updatePoseEstimate();

            NormalizedRGBA colors = intake.get();
            telemetry.addData("Red", "%.3f", colors.red)
                     .addData("Green", "%.3f", colors.green)
                     .addData("Blue", "%.3f", colors.blue);
            telemetry.update();

            intake.setServo(gamepad2.left_bumper);
        }
    }

}
