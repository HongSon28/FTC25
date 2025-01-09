package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystem.HangingSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private PinpointDrive drivetrain;
    private Intake intake;
    private Outtake outtake;
    private HangingSlide hangingSlide;
    private int slideState;
    private boolean outtakeSlideState,outtakeState;
    private int startPosID; // 0 = Closer to Net Zone, 1 = Closer to Observation Zone
    private boolean boost;

    @Override
    public void runOpMode() {
        startPosID = -1;
        while (startPosID == -1) {
            if (gamepad1.left_bumper) startPosID = 0;
            if (gamepad1.right_bumper) startPosID = 1;
        }

        drivetrain = new PinpointDrive(hardwareMap, RobotConfig.STARTING_POSE[startPosID]);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        hangingSlide = new HangingSlide(hardwareMap);
        slideState = 0;
        boost = false;
        outtakeSlideState = outtakeState = false;

        telemetry.addData("Status", "Ready for start");
        telemetry.addData("Starting Position ID", startPosID);
        telemetry.update();

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

            // For debugging
            telemetry.addData("Odometry Pod X", drivetrain.getEncoderX());
            telemetry.addData("Odometry Pod Y", drivetrain.getEncoderY());
            telemetry.addData("Current X", drivetrain.pose.position.x)
                     .addData("Current Y", drivetrain.pose.position.y)
                     .addData("Current Heading", drivetrain.pose.heading.toDouble());

            NormalizedRGBA colors = intake.get();
            telemetry.addData("Red", "%.3f", colors.red)
                     .addData("Green", "%.3f", colors.green)
                     .addData("Blue", "%.3f", colors.blue);
            telemetry.update();

            if (gamepad2.left_bumper) intake.setServo(2);
            else if (gamepad2.right_bumper) intake.setServo(1);
            else intake.setServo(0);

            if (gamepad2.dpad_up) slideState = 2;
            if (gamepad2.dpad_down) slideState = 1;
            if (gamepad2.dpad_right) {
                if (!boost) hangingSlide.boostPower();
                boost = true;
            }
            if (gamepad2.dpad_left) {
                if (boost) hangingSlide.resetPower();
                boost = false;
            }
            hangingSlide.moveToState(slideState);

            if (gamepad2.triangle) outtakeSlideState = true;
            if (gamepad2.cross) outtakeSlideState = false;
            if (gamepad2.circle) outtakeState = true;
            if (gamepad2.square) outtakeState = false;
            outtake.setSlide(outtakeSlideState);
            outtake.setServo(outtakeState);
        }
    }

}
