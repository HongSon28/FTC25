package org.firstinspires.ftc.teamcode.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DoubleSlide{
    private DcMotor leftSlideMotor ;
    private DcMotor rightSlideMotor;
    public DoubleSlide(HardwareMap hardwareMap){
        leftSlideMotor=hardwareMap.get(DcMotor.class,"leftSlideMotor");
        rightSlideMotor=hardwareMap. get(DcMotor .class,"rightSlideMotor");
        leftSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        rightSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    public void update(Gamepad gamepad,Telemetry telemetry) {
        double slidePower=-1*gamepad.left_stick_y;
        leftSlideMotor.setPower(slidePower);
        rightSlideMotor.setPower(slidePower);
        telemetry.addData("slide Power", slidePower);
        telemetry.addData ("slide Position:", leftSlideMotor.getCurrentPosition());
        telemetry.update() ;}}
