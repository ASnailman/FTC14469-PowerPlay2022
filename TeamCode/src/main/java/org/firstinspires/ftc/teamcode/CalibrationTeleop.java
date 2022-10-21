package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "CalibrationTeleop",  group = "MecanumDrive")
public class CalibrationTeleop extends LinearOpMode {

    //Control Hub Orientation
    byte AXIS_MAP_CONFIG_BYTE = 0x06; //rotates control hub 90 degrees around y axis by swapping x and z axis
    byte AXIS_MAP_SIGN_BYTE = 0x01; //Negates the remapped z-axis

    //Motors
    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor RailLeft;
    static DcMotor RailRight;
    static DcMotor RotatingBase;
    static CRServo RightClaw;
    static CRServo LeftClaw;

    //Sensors
    BNO055IMU IMU;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    int programOrder = 0;
    double movement;
    boolean PowerSetting = false;
    int increase;

    //Gyrocontinuity Variables
    double current_value;
    double prev_value = 0;
    double final_value;

    //Button Pressing Variables
    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_right_already_pressed2 = false;
    boolean button_dpad_left_already_pressed2 = false;

    public void runOpMode() {

        //Initialize the motors and sensors
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        RightClaw = hardwareMap.get(CRServo.class, "rightClaw");
        LeftClaw = hardwareMap.get(CRServo.class, "leftClaw");
        RailLeft = hardwareMap.get(DcMotor.class, "RailLeft");
        RailRight = hardwareMap.get(DcMotor.class, "RailRight");
        RotatingBase = hardwareMap.get(DcMotor.class, "RotatingBase");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //Configure the control hub orientation
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        //Attachment Presets
        AttachmentMotorPresets();

        //Claw Presets
        LeftClaw.setDirection(CRServo.Direction.REVERSE);
        LeftClaw.setPower(-1);
        RightClaw.setDirection(CRServo.Direction.FORWARD);
        RightClaw.setPower(-1);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
//        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            /*****************************************************************
             * Up Dpad (G1) - Set Low Power Mode/High Power Mode for driving
             *****************************************************************/

            if (PowerSetting) {
                movement = 0.7;
            } else {
                movement = 0.4;
            }

            if (!button_dpad_up_already_pressed) {
                if (gamepad1.dpad_up) {
                    if (!PowerSetting) {
                        PowerSetting = true;
                    } else {
                        PowerSetting = false;
                    }
                    button_dpad_up_already_pressed = true;
                }
            } else {
                if (!gamepad1.dpad_up) {
                    button_dpad_up_already_pressed = false;
                }
            }

            /*****************************************************************
             * Left/Right Analog Sticks (G1) - Movement
             *****************************************************************/

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

            /*****************************************************************
             * Dpad Left/Right (G2) - Calibrate Base
             *****************************************************************/
            if (!button_dpad_left_already_pressed2) {
                if (gamepad2.dpad_left) {
                    increase = increase - 25;
                    button_dpad_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_left) {
                    button_dpad_left_already_pressed2 = false;
                }
            }

            if (!button_dpad_right_already_pressed2) {
                if (gamepad2.dpad_right) {
                    increase = increase + 25;
                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Bumper Right (G2) : Rotate 360 base according to the "increase" in ticks
             *****************************************************************/

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {
                    RotatingBase.setTargetPosition(increase);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            telemetry.addData("Base Position", increase);
            telemetry.update();
        }
    }

    private double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;
    }

    public void AttachmentMotorPresets() {
        RailLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        RailLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RailLeft.setTargetPosition(0);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RailRight.setDirection(DcMotorSimple.Direction.FORWARD);
        RailRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RailRight.setTargetPosition(0);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RotatingBase.setDirection(DcMotorSimple.Direction.FORWARD);
        RotatingBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotatingBase.setTargetPosition(0);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SetRailPosition(int position) {
        RailRight.setTargetPosition(position);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailRight.setPower(1);
        RailLeft.setTargetPosition(position);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailLeft.setPower(1);
    }
}
