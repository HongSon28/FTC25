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
    private int allianceID; // 0 = Blue, 1 = Red;
    private boolean boost;
    private boolean locked;

    @Override
    public void runOpMode() {
        startPosID = allianceID = -1;
        while (startPosID == -1 || allianceID == -1) {
            if (gamepad1.left_bumper) startPosID = 0;
            if (gamepad1.right_bumper) startPosID = 1;
            if (gamepad1.cross) allianceID = 0;
            if (gamepad1.circle) allianceID = 1;
        }

        drivetrain = new PinpointDrive(hardwareMap, RobotConfig.STARTING_POSE[startPosID]);
        intake = new Intake(hardwareMap, allianceID);
        outtake = new Outtake(hardwareMap);
        hangingSlide = new HangingSlide(hardwareMap);
        slideState = 0;
        boost = false;
        locked = true;
        outtakeSlideState = outtakeState = false;

        telemetry.addData("Status", "Ready for start");
        telemetry.addData("Starting Position ID", startPosID);
        telemetry.addData("Alliance ID", allianceID);
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
            telemetry.addData("Sensor Output", intake.check());
            telemetry.update();

            //Intake control
            if (gamepad2.left_bumper) intake.setMotor(2);
            else if (gamepad2.right_bumper) intake.setMotor(1);
            else intake.setMotor(0);
            intake.setSlide(-gamepad2.left_stick_y);
            if (gamepad2.right_stick_button) locked = false;
            if (gamepad2.left_stick_button) locked = true;
            intake.setLock(locked);

            //Hanging Slides control
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

            //Outtake control
            if (gamepad2.triangle) outtakeSlideState = true;
            if (gamepad2.cross) outtakeSlideState = false;
            if (gamepad2.circle) outtakeState = true;
            if (gamepad2.square) outtakeState = false;
            outtake.setSlide(outtakeSlideState);
            outtake.setServo(outtakeState);
        }
    }

}
