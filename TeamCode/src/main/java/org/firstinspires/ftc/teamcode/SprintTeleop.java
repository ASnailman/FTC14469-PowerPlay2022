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

@TeleOp(name = "SprintTeleop",  group = "MecanumDrive")
public class SprintTeleop extends LinearOpMode {

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
    static CRServo LeftClaw;
    static CRServo RightClaw;

    //Sensors
    BNO055IMU IMU;

    //Variables of Classes
    Methods motorMethods;
    Mech_Drive_FAST MechDrive;
    Rail_Control RailControl;

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
    double movement;
    boolean PowerSetting = true;
    boolean ClawSetting = false;
    boolean coneStackMode = false;
    boolean groundJunctionMode = false;

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

    public void runOpMode() {

        //Initialize the motors and sensors
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        LeftClaw = hardwareMap.get(CRServo.class, "leftClaw");
        RightClaw = hardwareMap.get(CRServo.class, "rightClaw");
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
//        LeftClaw.scaleRange(0, 1);
//        LeftClaw.setPosition(0);
        LeftClaw.setPower(-1);
        RightClaw.setDirection(CRServo.Direction.FORWARD);
//        RightClaw.scaleRange(0, 1);
//        RightClaw.setPosition(0);
        RightClaw.setPower(-1);

        //Configrue IMU for GyroTurning
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        globalangle = 0;

        //Mechdrive Object
        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);
        RailControl = new Rail_Control(RailLeft, RailRight);
        //        motorMethods = new Methods(telemetry, IMU, orientation, FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.FORWARD);

        //Zero Power Behavior
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        ET.reset();

        while (opModeIsActive()) {

            /*****************************************************************
             * Up Dpad (G1) - Set Low Power Mode/High Power Mode for driving
             *****************************************************************/

            if (PowerSetting) {
                movement = 0.75;
            } else {
                movement = 0.3;
            }

            if (!button_bumper_right_already_pressed) {
                if (gamepad1.right_bumper) {
                    if (!PowerSetting) {
                        PowerSetting = true;
                    } else {
                        PowerSetting = false;
                    }
                    button_bumper_right_already_pressed = true;
                }
            } else {
                if (!gamepad1.right_bumper) {
                    button_bumper_right_already_pressed = false;
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
             * Dpad Up (G2) - Open/Close Claw
             *****************************************************************/

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
//                RightClaw.setPosition(0);
//                LeftClaw.setPosition(1);
                RightClaw.setPower(-1);
                LeftClaw.setPower(-1);
            } else {
//                RightClaw.setPosition(1);
//                LeftClaw.setPosition(0);
                RightClaw.setPower(1);
                LeftClaw.setPower(1);
            }

                switch (targetJunction) {
                    case 1:
                        ET.reset();
                        targetJunction++;
                        break;
                    case 2:
                        if (ET.milliseconds() > 800) {
                            RailControl.SetTargetPosition(2125, -1, 1);
                            SetBasePosition(0);
                            targetJunction++;
                        }
                        break;
                    case 3:
                        break;

                    case 4:
                        ET.reset();
                        targetJunction++;
                        break;
                    case 5:
                        if (ET.milliseconds() > 800) {
                            RailControl.SetTargetPosition(3475, -1, 1);
                            SetBasePosition(0);
                            targetJunction++;
                        }
                        break;
                    case 6:
                        break;

                    case 7:
                        ET.reset();
                        targetJunction++;
                        break;
                    case 8:
                        if (ET.milliseconds() > 800) {
                            RailControl.SetTargetPosition(4825, -1, 1);
                            SetBasePosition(0);
                            targetJunction++;
                        }
                        break;
                    case 9:
                        break;

                    case 10:
                        ET.reset();
                        targetJunction++;
                        break;

                    case 11:
                        if (ET.milliseconds() > 800) {
                            RailControl.SetTargetPosition(300, -0.5, 0.5);
                            SetBasePosition(0);
                            targetJunction++;
                        }
                        break;
                    case 12:
                        break;

                    default:
                        break;

                }

            //If cone stack mode is false, then buttons a, b, and y will correspond to the height of the junctions
            //If ground junction mode is true, then button a will not close claw when moving to low junction position
            if (!coneStackMode) {

                /*****************************************************************
                 * Button A (G2) : Set Rail height to place on the low junction
                 *****************************************************************/

                if (groundJunctionMode) {
                    if (!button_a_already_pressed2) {
                        if (gamepad2.a) {
                            //code for low junction when pressed
                            ClawSetting = false;
                            coneStackMode = false;
                            groundJunctionMode = false;
//                            RightClaw.setPosition(0);
//                            LeftClaw.setPosition(0);
                            RightClaw.setPower(-1);
                            LeftClaw.setPower(-1);
                            targetJunction = 1;
                            button_a_already_pressed2 = true;
                        }
                    } else {
                        if (!gamepad2.a) {
                            button_a_already_pressed2 = false;
                        }
                    }
                } else {
                    if (!button_a_already_pressed2) {
                        if (gamepad2.a) {
                            //code for low junction when pressed
                            ClawSetting = true;
                            coneStackMode = true;
//                            RightClaw.setPosition(1);
//                            LeftClaw.setPosition(1);
                            RightClaw.setPower(1);
                            LeftClaw.setPower(1);
                            targetJunction = 1;
                            button_a_already_pressed2 = true;
                        }
                    } else {
                        if (!gamepad2.a) {
                            button_a_already_pressed2 = false;
                        }
                    }
                }

                /*****************************************************************
                 * Button B (G2) : Set Rail height to place on the medium junction
                 *****************************************************************/

                if (!button_b_already_pressed2) {
                    if (gamepad2.b) {
                        //code for medium junction when pressed
                        ClawSetting = true;
                        coneStackMode = true;
//                        RightClaw.setPosition(1);
//                        LeftClaw.setPosition(1);
                        RightClaw.setPower(1);
                        LeftClaw.setPower(1);
                        targetJunction = 4;
                        button_b_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.b) {
                        button_b_already_pressed2 = false;
                    }
                }

                /*****************************************************************
                 * Button Y (G2) : Set Rail height to place on the high junction
                 *****************************************************************/

                if (!button_y_already_pressed2) {
                    if (gamepad2.y) {
                        //code for low junction when pressed
                        ClawSetting = true;
                        coneStackMode = true;
//                        RightClaw.setPosition(1);
//                        LeftClaw.setPosition(1);
                        RightClaw.setPower(1);
                        LeftClaw.setPower(1);
                        targetJunction = 7;
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }

            //if cone stack mode is true, buttons a, b, and y will correspond to the height of the cone stack
            } else {

                if (!button_a_already_pressed2) {
                    if (gamepad2.a) {
                        //code for low junction when pressed
                        ClawSetting = false;
                        coneStackMode = false;
//                        RightClaw.setPosition(0);
//                        LeftClaw.setPosition(0);
                        RightClaw.setPower(-1);
                        LeftClaw.setPower(-1);
                        RailControl.SetTargetPosition(2125, -1, 1);
                        SetBasePosition(0);
//                        SetRailPosition(535);
//                        SetBasePosition(-3763);
                        button_a_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.a) {
                        button_a_already_pressed2 = false;
                    }
                }

                if (!button_b_already_pressed2) {
                    if (gamepad2.b) {
                        //code for medium junction when pressed
                        ClawSetting = false;
                        coneStackMode = false;
//                        RightClaw.setPosition(1);
//                        LeftClaw.setPosition(1);
                        RightClaw.setPower(1);
                        LeftClaw.setPower(1);
                        RailControl.SetTargetPosition(800, -1, 1);
                        SetBasePosition(-3763);
                        button_b_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.b) {
                        button_b_already_pressed2 = false;
                    }
                }

                if (!button_y_already_pressed2) {
                    if (gamepad2.y) {
                        //code for low junction when pressed
                        ClawSetting = false;
                        coneStackMode = false;
//                        RightClaw.setPosition(1);
//                        LeftClaw.setPosition(1);
                        RightClaw.setPower(1);
                        LeftClaw.setPower(1);
                        RailControl.SetTargetPosition(900, -1, 1);
                        SetBasePosition(-3763);
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }
            }

            /*****************************************************************
             * Button X (G2) : Reset Rail and Base from any height to original position
             *****************************************************************/

            if (!button_x_already_pressed2) {
                if (gamepad2.x) {
                    //code for releasing cone and resetting base
                    ClawSetting = false;
                    coneStackMode = false;
                    groundJunctionMode = false;
//                    RightClaw.setPosition(0);
//                    LeftClaw.setPosition(0);
                    RightClaw.setPower(-1);
                    LeftClaw.setPower(-1);
                    RailControl.SetTargetPosition(0, -0.7, 0.7);
                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
                }
            }

            if (!button_dpad_right_already_pressed2) {
                if (gamepad2.dpad_right) {
                    //code for releasing cone and resetting base
                    ClawSetting = true;
                    coneStackMode = false;
                    groundJunctionMode = false;
//                    RightClaw.setPosition(0);
//                    LeftClaw.setPosition(0);
                    RightClaw.setPower(1);
                    LeftClaw.setPower(1);
                    RailControl.SetTargetPosition(0, -0.7, 0.7);
                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Button Dpad Down (G2) : Set Rail For Ground Junction
             *****************************************************************/

            if (!button_dpad_down_already_pressed2) {
                if (gamepad2.dpad_down) {
                    //code for releasing cone and resetting base
                    ClawSetting = true;
                    coneStackMode = false;
                    groundJunctionMode = true;
//                    RightClaw.setPosition(1);
//                    LeftClaw.setPosition(1);
                    RightClaw.setPower(1);
                    LeftClaw.setPower(1);
                    targetJunction = 10;
                    button_dpad_down_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_down) {
                    button_dpad_down_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Bumper Right (G2) : Rotate 360 base 90 degrees
             *****************************************************************/

            if (!button_bumper_right_already_pressed2) {
                if (gamepad2.right_bumper) {
                    RotatingBase.setTargetPosition(-3763);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Bumper Left (G2) : Rotate 360 base to -90
             *****************************************************************/

            if (!button_bumper_left_already_pressed2) {
                if (gamepad2.left_bumper) {
                    RotatingBase.setTargetPosition(3763);
                    RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RotatingBase.setPower(1);
                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            RailControl.RailTask();
            telemetry.addData("LeftRail Power", RailLeft.getPower());
            telemetry.addData("LeftRail Encoder", RailLeft.getCurrentPosition());
            telemetry.addData("RightRail Power", RailRight.getPower());
            telemetry.addData("RightRail Encoder", RailRight.getCurrentPosition());
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
        RailLeft.setTargetPosition(-position);
        RailLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RailLeft.setPower(1);
    }

    public void SetBasePosition(int position) {
        RotatingBase.setTargetPosition(position);
        RotatingBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RotatingBase.setPower(1);
    }
}
