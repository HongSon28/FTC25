package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystem.HangingSlide;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends LinearOpMode {
    private PinpointDrive drivetrain;
    private Intake intake;
    private Outtake outtake;
//    private HangingSlide hangingSlide;
    private int slideState, liftState;
    private boolean intakeSlideState;
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
        outtake = new Outtake(hardwareMap,telemetry);
//        hangingSlide = new HangingSlide(hardwareMap);
        slideState = liftState = 0;
        boost = false;
        locked = false;
        intakeSlideState = false;

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
            /*
            telemetry.addData("Odometry Pod X", drivetrain.getEncoderX());
            telemetry.addData("Odometry Pod Y", drivetrain.getEncoderY());
            telemetry.addData("Current X", drivetrain.pose.position.x)
                     .addData("Current Y", drivetrain.pose.position.y)
                     .addData("Current Heading", drivetrain.pose.heading.toDouble());
            telemetry.addData("Current Angle",intake.curPos);
            telemetry.update();*/

            //Intake control (using bumpers and sticks)
            if (gamepad2.left_bumper) intake.setServo(2);
            else if (gamepad2.right_bumper) intake.setServo(1);
            else intake.setServo(0);
            if (gamepad1.left_trigger > 0.1 && outtake.armState < 2) intakeSlideState = true;
            else if (gamepad1.right_trigger > 0.1) {
                intakeSlideState = false;
            }
            intake.setSlide(intakeSlideState);

            if (gamepad2.dpad_down) intake.setAngle(true);
            else if (gamepad2.dpad_up) intake.setAngle(false);

//            if (gamepad2.touchpad) liftState = 0;
//            if (gamepad2.right_stick_button) liftState = 1;
//            if (gamepad2.left_stick_button) liftState = 2;
//            intake.setLift(liftState);

            //Hanging Slides control (using dpad buttons)
//            if (gamepad2.dpad_right) {
//                if (!boost) hangingSlide.boostPower();
//                boost = true;
//            }
//            if (gamepad2.dpad_left) {
//                if (boost) hangingSlide.resetPower();
//                boost = false;
//            }
//            hangingSlide.moveToState(slideState);

            //Outtake  control (using shapes buttons)
            if (gamepad2.triangle) outtake.setSlide(true);
            if (gamepad2.cross) outtake.setSlide(false);
            if (gamepad2.circle) {
                if (outtake.armState == 1) {
                    if (!intake.extended) outtake.setArm(1);
                } else outtake.setArm(1);
            }
            else if (gamepad2.square) outtake.setArm(-1);
            else outtake.setArm(0);
            outtake.setWrist(gamepad2.right_stick_button);

//            if (gamepad1.right_trigger > 0.1) outtake.setWrist(1);
//            else if (gamepad1.left_trigger > 0.1) outtake.setWrist(-1);
//            else outtake.setWrist(0);

            //Claw control
            if (gamepad2.left_trigger > 0.25) outtake.setClaw(false);
            if (gamepad2.right_trigger > 0.25) outtake.setClaw(true);

            outtake.control(telemetry);

        }
    }

}
