package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleopTest",  group = "MecanumDrivetoo")
public class TeleopTest extends LinearOpMode {

    //Control Hub Orientation
    byte AXIS_MAP_CONFIG_BYTE = 0x06; //rotates control hub 90 degrees around y axis by swapping x and z axis
    byte AXIS_MAP_SIGN_BYTE = 0x01; //Negates the remapped z-axis

    //Motors
    static DcMotor RailLeft;
    static DcMotor RailRight;
    static DcMotor ExtendingRail;
    static CRServo Claw;

    //Sensors
    BNO055IMU IMU;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;
//    Rail_Control RailControl;
    Rail_ControlV2 RailControlV2;

    //Variables For IMU Gyro
    double globalangle;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;

    //Variables
    ElapsedTime ET = new ElapsedTime();
    int leftHighPickup = 0;
    int rightHighPickup = 0;
    int resetSequence = 0;
    int targetJunction = 0;
    int baseManualReset = 0;
    double movement;
    boolean lowJunctionResetMode;
    boolean PowerSetting = true;
    boolean ClawSetting = false;
    boolean coneStackMode = false;
    boolean groundJunctionMode = false;
    int basePosition;

    int extendingOrder;

    double l;
    double assist_gain = 0.02;
    double assist_offset = 0.05;    // compensation - robot drifting to the right when base is 0 deg

    //Gyrocontinuity Variables
    double current_value;
    double prev_value = 0;
    double final_value;

    //Button Pressing Variables
    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_right_already_pressed2 = false;
    boolean button_dpad_left_already_pressed2 = false;
    boolean button_dpad_up_already_pressed2 = false;
    boolean button_dpad_down_already_pressed2 = false;
    boolean button_left_trigger_already_pressed2 = false;
    boolean button_right_trigger_already_pressed2 = false;
    boolean double_trigger_already_pressed = false;

    public void runOpMode() {

        //Initialize the motors and sensors
        RailLeft = hardwareMap.get(DcMotor.class, "RailLeft");
        RailRight = hardwareMap.get(DcMotor.class, "RailRight");
//        ExtendingRail = hardwareMap.get(DcMotor.class, "ExtendingRail");
        Claw = hardwareMap.get(CRServo.class, "Claw");
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

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        Claw.setDirection(CRServo.Direction.FORWARD);
//        Claw.setPower(-1);

        //Mechdrive Object
        RailControlV2 = new Rail_ControlV2(RailLeft, RailRight);
        //        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        waitForStart();

        ET.reset();

        while (opModeIsActive()) {

            if (!button_dpad_up_already_pressed2) {
                if (gamepad2.dpad_up) {
                    if (!ClawSetting) {
                        ClawSetting = true;
                    } else {
                        ClawSetting = false;
                    }
                    button_dpad_up_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_up) {
                    button_dpad_up_already_pressed2 = false;
                }
            }

            if (!ClawSetting) {
                Claw.setPower(1);
            } else {
                Claw.setPower(-1);
            }

            if (!button_x_already_pressed2) {
                    if (gamepad2.x) {
                        RailControlV2.SetTargetPosition(0, -1, 1);
//                        SetRailPosition(0, 1);
                        button_x_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.x) {
                        button_x_already_pressed2 = false;
                    }
                }

                if (!button_a_already_pressed2) {
                    if (gamepad2.a) {
                        RailControlV2.SetTargetPosition(1910, -1, 1);
//                        SetRailPosition(1910, 0.85);
                        button_a_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.a) {
                        button_a_already_pressed2 = false;
                    }
                }

                if (!button_b_already_pressed2) {
                    if (gamepad2.b) {
                        RailControlV2.SetTargetPosition(2914, -1, 1);
//                        SetRailPosition(2914, 0.85);
                        button_b_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.b) {
                        button_b_already_pressed2 = false;
                    }
                }

                if (!button_y_already_pressed2) {
                    if (gamepad2.y) {
                        RailControlV2.SetTargetPosition(4146, -1, 1);
//                        SetRailPosition(4146, 0.85);
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }

//            if (!button_bumper_right_already_pressed2) {
//                if (gamepad2.right_bumper) {
//                    //
//                    ExtendingRail.setTargetPosition(400);
//                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ExtendingRail.setPower(0.3);
//                    button_bumper_right_already_pressed2 = true;
//                }
//            } else {
//                if (!gamepad2.right_bumper) {
//                    button_bumper_right_already_pressed2 = false;
//                }
//            }
//
//            if (!button_bumper_left_already_pressed2) {
//                if (gamepad2.left_bumper) {
//                    ExtendingRail.setTargetPosition(0);
//                    ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ExtendingRail.setPower(0.3);
//                    button_bumper_left_already_pressed2 = true;
//                }
//            } else {
//                if (!gamepad2.left_bumper) {
//                    button_bumper_left_already_pressed2 = false;
//                }
//            }

//            RailControl.RailTask();
            RailControlV2.RailTask();
//            telemetry.addData("LeftRail Power", RailLeft.getPower());
            telemetry.addData("LeftRail Encoder", RailLeft.getCurrentPosition());
//            telemetry.addData("RightRail Power", RailRight.getPower());
            telemetry.addData("RightRail Encoder", RailRight.getCurrentPosition());
//            telemetry.addData("Extending Rail Encoder", ExtendingRail.getCurrentPosition());
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

//        ExtendingRail.setDirection(DcMotorSimple.Direction.FORWARD);
//        ExtendingRail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ExtendingRail.setTargetPosition(0);
//        ExtendingRail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void SetRailPosition(int position, double power) {
        RailRight.setTargetPosition(-position);
        RailRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailRight.setPower(power);
        RailLeft.setTargetPosition(position);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailLeft.setPower(power);
    }
}
