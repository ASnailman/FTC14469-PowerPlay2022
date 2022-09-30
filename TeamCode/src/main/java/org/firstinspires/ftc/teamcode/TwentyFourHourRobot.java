package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "24HourRobot", group = "MecanumDrive")
public class TwentyFourHourRobot extends LinearOpMode {

    byte AXIS_MAP_CONFIG_BYTE = 0x6; //rotates control hub 90 degrees around y axis by swapping x and z axis

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Rail;
    static Servo Claw;
    BNO055IMU IMU;
    boolean open;
    double movement = 0.7;
    int increase = 0;

    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_bumper_left_already_pressed2 = false;

    @Override
    public void runOpMode() {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Rail = hardwareMap.get(DcMotor.class, "Rail");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        Rail.setDirection(DcMotorSimple.Direction.FORWARD);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setTargetPosition(0);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DCMotorPresets();

        Claw.setDirection(Servo.Direction.FORWARD);
        Claw.scaleRange(0, 1);
        Claw.setPosition(0.2);

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y * movement;
            double x = gamepad1.left_stick_x * movement;
            double rx = gamepad1.right_stick_x * movement;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            FrontLeft.setPower(FLPower);
            BackLeft.setPower(BLPower);
            FrontRight.setPower(FRPower);
            BackRight.setPower(BRPower);

            if (gamepad1.dpad_right) {
                movement = 0.4;
            }
            if (gamepad1.dpad_left) {
                movement = 0.7;
            }

            if (gamepad2.dpad_left) {

                //code for the claw to close
                Claw.setPosition(1);
            }

            if (gamepad2.dpad_right) {
                //code for the claw to open
                Claw.setPosition(0);
            }

            if (gamepad2.a) {
                //code for low junction when pressed
                Claw.setPosition(1);
                sleep(400);
                Rail.setTargetPosition(4200);
                Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rail.setPower(1);
            }

            if (gamepad2.b) {
                //code for middle junction when pressed
                Claw.setPosition(1);
                sleep(400);
                Rail.setTargetPosition(7000);
                Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rail.setPower(1);
            }

            if (gamepad2.y) {
                //code for high junction when pressed
                Claw.setPosition(1);
                sleep(400);
                Rail.setTargetPosition(9520);
                Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rail.setPower(1);
            }

            if (gamepad2.x) {
                //code for releasing cone on any junction
                Claw.setPosition(0.2);
                Rail.setTargetPosition(0);
                Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rail.setPower(1);

            }

            if (!button_bumper_left_already_pressed2) {
                if (gamepad2.left_bumper) {
                    //code for releasing cone on any junction
                    increase = increase - 5;
                    Rail.setTargetPosition(increase);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(1);
                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (gamepad2.left_bumper) {
                    //code for releasing cone on any junction
                    increase = increase - 5;
                    Rail.setTargetPosition(increase);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(1);
                    button_bumper_left_already_pressed2 = false;
                }
            }

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {
                    //code for releasing cone on any junction
                    increase = increase + 5;
                    Rail.setTargetPosition(increase);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(1);
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (gamepad2.right_bumper) {
                    //code for releasing cone on any junction
                    increase = increase + 5;
                    Rail.setTargetPosition(increase);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(1);
                    button_bumper_right_already_pressed2 = false;
                }
            }


        }

    }

    public void DCMotorPresets() {
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
