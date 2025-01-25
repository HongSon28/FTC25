package org.firstinspires.ftc.teamcode.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
public class RadarServo {
    private Servo radarServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final double fx=578.272;
    private final double fy= 578.272;
    private final double cx= 402.145;
    private final double cy= 221.506;
    private static final double ServoMinPosition=0.0;
    private static final double ServoMaxPosition=1.0;
    private static final double ServoStepsTaken = 0.005; // lượng distance moi loop servo di chuyen em de 0.005 cho no quay muot
    private double servoPosition = 0.0; //gia tri hien tai cua servo [0;1]
    private boolean movingForward = true;
    public RadarServo( HardwareMap hardwareMap,Telemetry telemetry) {
        radarServo = hardwareMap.get(Servo.class,"radarServo");
        aprilTagProcessor=new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
        VisionPortal.Builder builder=new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam so 1"))
                .addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
        telemetry.addLine("RadarSubsystem init completed bro");
        telemetry.update();
    }
     //Called periodically (inside a while(opModeIsActive()) loop)
    public void update(Telemetry telemetry){
        List<AprilTagDetection> detections= aprilTagProcessor.getDetections();
        int numTags= detections.size();
        if (numTags >= 1) {radarServo.setPosition(servoPosition);
            AprilTagDetection firstTag = detections.get(0);
            double angleDegrees = servoPosition*300.0;
            telemetry.addLine("Detected " + numTags + "AprilTag so stopping servo bro");
            telemetry.addData("First Tag ID",firstTag.id);
            telemetry.addData("Servo Angle (deg)",angleDegrees);

        }else {sweepServo();
            telemetry.addLine("No Apriltags detected ->still scanning ");
        }}

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    private void sweepServo(){ //dcm cach scan servo hoi ngu nma em cung dbt lam the nao nua :))))
        if(movingForward){
            servoPosition=servoPosition+ ServoStepsTaken;
            if (servoPosition>= ServoMaxPosition){
                servoPosition= ServoMaxPosition;
                movingForward= false;}} else {
            servoPosition=servoPosition- ServoStepsTaken;
            if (servoPosition<=ServoMinPosition) {
                servoPosition =ServoMinPosition;
                movingForward= true;
            }}radarServo.setPosition(servoPosition);}
}
