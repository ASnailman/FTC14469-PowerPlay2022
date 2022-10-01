package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Sprint2Auto", group = "MecanumDrive")
public class Sprint2Auto extends LinearOpMode {

    byte AXIS_MAP_CONFIG_BYTE = 0x21; //swap y and z axis
    byte AXIS_MAP_SIGN_BYTE = 0x07; //negate z axis

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Rail;
    static Servo Claw;
    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;

    int programOrder = 0;

//    OpenCvWebcam webcam;
//    BarcodeDeterminationPipeline pipeline;

    Mech_Drive_FAST MechDrive;

    static int DifferenceLeft;
    static int DifferenceCenter;
    static int DifferenceRight;

    boolean turnright = false;
    boolean turnleft = false;

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
        IMU.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        Rail.setDirection(DcMotorSimple.Direction.FORWARD);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setTargetPosition(0);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        DCMotorPresets();

        Claw.setDirection(Servo.Direction.FORWARD);
        Claw.scaleRange(0, 1);
        Claw.setPosition(0.2);

        MechDrive = new Mech_Drive_FAST(FrontRight, FrontLeft, BackRight, BackLeft, MoveDirection.FORWARD, telemetry);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);
        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        pipeline = new BarcodeDeterminationPipeline();
//        webcam.setPipeline(pipeline);
//        pipeline.InitTelemetry(telemetry);
//
//        webcam.setMillisecondsPermissionTimeout(2500);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });

        waitForStart();

        while (opModeIsActive()) {

            switch (programOrder) {

                case 0:
//                    MechDrive.SetTargets(0, 1000, 0.7, 0);
                    GyroTurn(45, 0.5);
                    programOrder++;
                    break;

                default:
                    break;
            }

//            MechDrive.Task(GyroContinuity());
//            telemetry.addData("backright encoder", BackRight.getCurrentPosition());
            telemetry.addData("gyro", GyroContinuity());
            telemetry.update();
        }

    }

    private void GyroTurn (double angledegree, double power) {

        if (angledegree > GyroContinuity()) {

            if (GyroContinuity() < angledegree) {
                MotorTurn(-power, power, -power, power);
                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();
            } else {
//                programorder1++;
                SetMotorPower(0);

                while (FrontRight.getCurrentPosition() != 0) {
                    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }
        else if (angledegree < GyroContinuity()) {

            if (GyroContinuity() > angledegree) {
                MotorTurn(power, -power, power, -power);
                telemetry.addData("Gyro", GyroContinuity());
                telemetry.update();
            } else {
//                programorder1++;
                SetMotorPower(0);

                while (FrontRight.getCurrentPosition() != 0) {
                    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
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

    private void SetMotorPower(double x) {

        FrontLeft.setPower(x);
        FrontRight.setPower(x);
        BackLeft.setPower(x);
        BackRight.setPower(x);

    }

    private void MotorTurn(double FR, double FL, double BR, double BL) {
        FrontRight.setPower(FR);
        FrontLeft.setPower(FL);
        BackRight.setPower(BR);
        BackLeft.setPower(BL);
    }

//    public static class BarcodeDeterminationPipeline extends OpenCvPipeline
//    {
//        /*
//         * An enum to define the skystone position
//         */
//        public enum ShippingElementPosition
//        {
//            NONE,
//            LEFT,
//            CENTER,
//            RIGHT
//        }
//
//        /*
//         * Some color constants
//         */
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//        static final Scalar RED = new Scalar(255, 0, 0);
//        static final Scalar YELLOW = new Scalar(255, 255, 0);
//
//        /*
//         * The core values which define the location and size of the sample regions
//         */
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(60,85);
//        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(590,65);
//        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1050,65);
//        static final int REGION_WIDTH = 80;
//        static final int REGION_HEIGHT = 80;
//
//        static final int SHIPPING_ELEMENT_THRESHOLD = 55;
//
//        Telemetry telemetry_vision;
//
//        /*
//         * Points which actually define the sample region rectangles, derived from above values
//         *
//         * Example of how points A and B work to define a rectangle
//         *
//         *   ------------------------------------
//         *   | (0,0) Point A                    |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                  Point B (70,50) |
//         *   ------------------------------------
//         *
//         */
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//        Point region2_pointA = new Point(
//                REGION2_TOPLEFT_ANCHOR_POINT.x,
//                REGION2_TOPLEFT_ANCHOR_POINT.y);
//        Point region2_pointB = new Point(
//                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//        Point region3_pointA = new Point(
//                REGION3_TOPLEFT_ANCHOR_POINT.x,
//                REGION3_TOPLEFT_ANCHOR_POINT.y);
//        Point region3_pointB = new Point(
//                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        /*
//         * Working variables
//         */
//        Mat region1_Cb, region2_Cb, region3_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1, avg2, avg3;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        //private volatile FarBlueOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition position = FarBlueOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition.LEFT;
//        private volatile ShippingElementPosition position = ShippingElementPosition.NONE;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 2);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            /*
//             * We need to call this in order to make sure the 'Cb'
//             * object is initialized, so that the submats we make
//             * will still be linked to it on subsequent frames. (If
//             * the object were to only be initialized in processFrame,
//             * then the submats would become delinked because the backing
//             * buffer would be re-allocated the first time a real frame
//             * was crunched)
//             */
//            inputToCb(firstFrame);
//
//            /*
//             * Submats are a persistent reference to a region of the parent
//             * buffer. Any changes to the child affect the parent, and the
//             * reverse also holds true.
//             */
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            /*
//             * Overview of what we're doing:
//             *
//             * We first convert to YCrCb color space, from RGB color space.
//             * Why do we do this? Well, in the RGB color space, chroma and
//             * luma are intertwined. In YCrCb, chroma and luma are separated.
//             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
//             * are Y, the luma channel (which essentially just a B&W image), the
//             * Cr channel, which records the difference from red, and the Cb channel,
//             * which records the difference from blue. Because chroma and luma are
//             * not related in YCrCb, vision code written to look for certain values
//             * in the Cr/Cb channels will not be severely affected by differing
//             * light intensity, since that difference would most likely just be
//             * reflected in the Y channel.
//             *
//             * After we've converted to YCrCb, we extract just the 2nd channel, the
//             * Cb channel. We do this because stones are bright yellow and contrast
//             * STRONGLY on the Cb channel against everything else, including SkyStones
//             * (because SkyStones have a black label).
//             *
//             * We then take the average pixel value of 3 different regions on that Cb
//             * channel, one positioned over each stone. The brightest of the 3 regions
//             * is where we assume the SkyStone to be, since the normal stones show up
//             * extremely darkly.
//             *
//             * We also draw rectangles on the screen showing where the sample regions
//             * are, as well as drawing a solid rectangle over top the sample region
//             * we believe is on top of the SkyStone.
//             *
//             * In order for this whole process to work correctly, each sample region
//             * should be positioned in the center of each of the first 3 stones, and
//             * be small enough such that only the stone is sampled, and not any of the
//             * surroundings.
//             */
//
//            /*
//             * Get the Cb channel of the input frame after conversion to YCrCb
//             */
//            inputToCb(input);
//
//            /*
//             * Compute the average pixel value of each submat region. We're
//             * taking the average of a single channel buffer, so the value
//             * we need is at index 0. We could have also taken the average
//             * pixel value of the 3-channel image, and referenced the value
//             * at index 2 here.
//             */
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//            avg2 = (int) Core.mean(region2_Cb).val[0];
//            avg3 = (int) Core.mean(region3_Cb).val[0];
//
//            /*
//             * Draw a rectangle showing sample region 1 on the screen.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    4); // Thickness of the rectangle lines
//
//            /*
//             * Draw a rectangle showing sample region 2 on the screen.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region2_pointA, // First point which defines the rectangle
//                    region2_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    4); // Thickness of the rectangle lines
//
//            /*
//             * Draw a rectangle showing sample region 3 on the screen.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region3_pointA, // First point which defines the rectangle
//                    region3_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    4); // Thickness of the rectangle lines
//
//            /*
//             * Now that we found the max, we actually need to go and
//             * figure out which sample region that value was from
//             */
//
//            DifferenceLeft = Avg1() - SHIPPING_ELEMENT_THRESHOLD;
//            DifferenceCenter = Avg2() - SHIPPING_ELEMENT_THRESHOLD;
//            DifferenceRight = Avg3() - SHIPPING_ELEMENT_THRESHOLD;
//
//            if ((DifferenceLeft > -40) && (DifferenceLeft < 40)) { // Was it from region 1?
//
//                position = ShippingElementPosition.LEFT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        4); // Negative thickness means solid fill
//            }
//
//            else if ((DifferenceCenter > -40) && (DifferenceCenter < 40)) { // Was it from region 2?
//
//                position = ShippingElementPosition.CENTER; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region2_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        4); // Negative thickness means solid fill
//            }
//
//            else if ((DifferenceRight > -40) && (DifferenceRight < 40)) { // Was it from region 3?
//
//                position = ShippingElementPosition.RIGHT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region3_pointA, // First point which defines the rectangle
//                        region3_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        4); // Negative thickness means solid fill
//            }
//
//            else {
//                position = ShippingElementPosition.NONE;
//            }
//
//            telemetry_vision.addData("Avg1", Avg1());
//            telemetry_vision.addData("Avg2", Avg2());
//            telemetry_vision.addData("Avg3", Avg3());
//            telemetry_vision.addData("Position", getAnalysis());
//            telemetry_vision.update();
//
//            /*
//             * Render the 'input' buffer to the viewport. But note this is not
//             * simply rendering the raw camera feed, because we called functions
//             * to add some annotations to this buffer earlier up.
//             */
//            return input;
//        }
//
//        public int Avg1 () {
//            return avg1;
//        }
//
//        public int Avg2 () {
//            return avg2;
//        }
//
//        public int Avg3 () {
//            return avg3;
//        }
//
//        /*
//         * Call this from the OpMode thread to obtain the latest analysis
//         */
//
//        public ShippingElementPosition getAnalysis()
//        {
//            return position;
//        }
//
//        public void InitTelemetry(Telemetry Obj) {
//            telemetry_vision = Obj;
//        }
//    }

}
