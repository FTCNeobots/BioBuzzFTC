package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "FirstClass")
public class FirstClass extends OpMode {

    //stores the values of the speeds the motor has to move at in this order: LF, LB, RF, RB
    double[] motorSpeeds = new double[4];
    double maxSpeed = 1;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    GoBildaPinpointDriver pinpoint;
    double robotHeading;


    @Override
    public void init() {
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        leftFront = hardwareMap.get(DcMotor.class, "lf");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        pinpoint.update();
        robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        telemetry.addData("Heading =", robotHeading);

        if(gamepad1.back){
            pinpoint.resetPosAndIMU();
        }

        motorSpeeds = MDT.Mecanum(maxSpeed, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robotHeading);
        leftFront.setPower(motorSpeeds[0]);leftBack.setPower(motorSpeeds[1]);rightFront.setPower(motorSpeeds[2]);rightBack.setPower(motorSpeeds[3]);

    }
}