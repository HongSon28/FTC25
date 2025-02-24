package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;

@Autonomous(name = "AutonomousOpMode")
public class AutonomousOpMode extends LinearOpMode {
    private PinpointDrive drivetrain;
    private Intake intake;
    private Outtake outtake;
    private int startPosID; // 0 = Closer to Net Zone, 1 = Closer to Observation Zone
    private int allianceID;

    @Override
    public void runOpMode() {
        startPosID = 1;
        allianceID = 2;

        drivetrain = new PinpointDrive(hardwareMap, new Pose2d(-25, 62, Math.toRadians(270)));
        intake = new Intake(hardwareMap, allianceID);
        outtake = new Outtake(hardwareMap,telemetry);

        waitForStart();

        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .splineTo(new Vector2d(-52,56),Math.toRadians(90))
                .build());

    }
}
