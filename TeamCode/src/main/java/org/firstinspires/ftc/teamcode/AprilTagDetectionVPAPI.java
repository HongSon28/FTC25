package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="AprilTagDetectionVPAPI", group="Linear Opmode")
public class AprilTagDetectionVPAPI extends LinearOpMode
{
    private CRServo radarServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    double ServoPower = 0.5; //gia tri thu nghiem, can thay doi
    double fx = 578.272; // gia tri thu nghiem, can thay doi
    double fy = 578.272; // gia tri thu nghiem, can thay doi
    double cx = 402.145; // gia tri thu nghiem, can thay doi
    double cy = 221.506; // gia tri thu nghiem, can thay doi

    @Override
    public void runOpMode()
    {
        radarServo = hardwareMap.get(CRServo.class, "radarServo");
        radarServo.setDirection(CRServo.Direction.FORWARD);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam so 1"))

                .addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        telemetry.addLine("loading. press start motherfucker...");
        telemetry.update();


        waitForStart();


        radarServo.setPower(ServoPower);

        while (opModeIsActive())
        {

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            if (!detections.isEmpty())
            {
                radarServo.setPower(0.0);
                telemetry.addLine("AprilTag DETECTED -> servo stopped");
            }
            else
            {
                radarServo.setPower(ServoPower);
                telemetry.addLine("No AprilTag -> servo spinning");
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}
