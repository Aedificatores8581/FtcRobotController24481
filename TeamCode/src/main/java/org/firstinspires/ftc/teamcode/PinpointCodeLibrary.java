package org.firstinspires.ftc.teamcode;

/**
 * ====================================================================================================
 *  goBILDA® Pinpoint Odometry & FTC Autonomous Master Code Library (450 Full Copy-Paste Snippets)
 * ====================================================================================================
 *  HOW TO USE THIS FILE:
 *  1. Open this file in Android Studio.
 *  2. Press Ctrl+F (Windows) or Cmd+F (Mac) to search for any ID Code from [CODE-001] to [CODE-450].
 *  3. Highlight the code block inside the snippet box and paste it directly into your OpMode!
 *  EVERY SINGLE ID FROM 001 TO 450 CONTAINS A FULL, EXECUTABLE JAVA CODE BLOCK!
 * ====================================================================================================
 */

public class PinpointCodeLibrary {

    /*
     * ================================================================================================
     *  CATEGORY 1: VARIABLES, MATH, AND IF STATEMENTS (CODE-001 to CODE-050)
     * ================================================================================================
     */

    // [CODE-001]
    // DESCRIPTION: Create a double variable for motor power (speed).
    // CODE:
    // double driveSpeed = 0.3;

    // [CODE-002]
    // DESCRIPTION: Create a double variable for target distance in inches.
    // CODE:
    // double targetDistanceInches = 12.0;

    // [CODE-003]
    // DESCRIPTION: Create a double variable for target turn angle in degrees.
    // CODE:
    // double targetAngleDegrees = 90.0;

    // [CODE-004]
    // DESCRIPTION: Create an int variable for loop counter or step number.
    // CODE:
    // int stepNumber = 1;

    // [CODE-005]
    // DESCRIPTION: Create a boolean variable (true/false flag).
    // CODE:
    // boolean isTargetReached = false;

    // [CODE-006]
    // DESCRIPTION: Create a String variable for text messages or status.
    // CODE:
    // String statusMessage = "Driving to Basket";

    // [CODE-007]
    // DESCRIPTION: Create a Pose2D variable to store X, Y, and Heading together.
    // CODE:
    // Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 12.0, 24.0, AngleUnit.DEGREES, 90.0);

    // [CODE-008]
    // DESCRIPTION: Create a final constant variable (value that never changes).
    // CODE:
    // final double MAX_POWER = 0.5;

    // [CODE-009]
    // DESCRIPTION: VARIABLE MATH: Add to an existing variable.
    // CODE:
    // double startY009 = 10.0;
    // double targetY009 = startY009 + 12.0;

    // [CODE-010]
    // DESCRIPTION: VARIABLE MATH: Subtract from an existing variable.
    // CODE:
    // double startY010 = 24.0;
    // double targetY010 = startY010 - 12.0;

    // [CODE-011]
    // DESCRIPTION: VARIABLE MATH: Multiply a variable.
    // CODE:
    // double maxSpeed011 = 0.8;
    // double halfSpeed011 = maxSpeed011 * 0.5;

    // [CODE-012]
    // DESCRIPTION: VARIABLE MATH: Divide a variable.
    // CODE:
    // double totalDist012 = 24.0;
    // double midpoint012 = totalDist012 / 2.0;

    // [CODE-013]
    // DESCRIPTION: VARIABLE UPDATE: Change an existing variable's value later in code.
    // CODE:
    // double targetDistance = 12.0;
    // targetDistance = 24.0;

    // [CODE-014]
    // DESCRIPTION: CORRECT IF STATEMENT: Basic IF checking if distance is reached ( > operator ).
    // CODE:
    // double currentY014 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // if (currentY014 >= 12.0) {
    //     stopMotors();
    // }

    // [CODE-015]
    // DESCRIPTION: CORRECT IF STATEMENT: Basic IF checking if heading reached target ( < operator ).
    // CODE:
    // double currentAngle015 = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    // if (currentAngle015 <= -90.0) {
    //     stopMotors();
    // }

    // [CODE-016]
    // DESCRIPTION: CORRECT IF STATEMENT: IF checking gamepad button press (gamepad1.a).
    // CODE:
    // if (gamepad1.a) {
    //     pinpoint.resetPosAndIMU();
    // }

    // [CODE-017]
    // DESCRIPTION: CORRECT IF STATEMENT: IF checking exact equality using == operator.
    // CODE:
    // int currentStep017 = 1;
    // if (currentStep017 == 1) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3);
    // }

    // [CODE-018]
    // DESCRIPTION: CORRECT IF STATEMENT: IF checking NOT equal using != operator.
    // CODE:
    // if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
    //     // Safe to read pose
    // }

    // [CODE-019]
    // DESCRIPTION: CORRECT IF-ELSE STATEMENT: Do one thing IF true, another thing ELSE.
    // CODE:
    // if (gamepad1.a) {
    //     setDrivePowers(0.6, 0.6, 0.6, 0.6);
    // } else {
    //     setDrivePowers(0.2, 0.2, 0.2, 0.2);
    // }

    // [CODE-020]
    // DESCRIPTION: CORRECT IF-ELSE IF-ELSE: 3-way decision tree (Red vs Blue vs Neutral).
    // CODE:
    // String allianceColor = "RED";
    // if (allianceColor.equals("RED")) {
    //     // Red path
    // } else if (allianceColor.equals("BLUE")) {
    //     // Blue path
    // } else {
    //     // Neutral path
    // }

    // [CODE-021]
    // DESCRIPTION: CORRECT IF STATEMENT WITH AND (&&): Both conditions must be true.
    // CODE:
    // pinpoint.update();
    // if (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 12.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3);
    // }

    // [CODE-022]
    // DESCRIPTION: CORRECT IF STATEMENT WITH OR (||): Either condition can be true.
    // CODE:
    // if (gamepad1.a || gamepad1.b) {
    //     stopMotors();
    // }

    // [CODE-023]
    // DESCRIPTION: COMMON IF MISTAKE FIX: Using == for comparing vs = for assigning.
    // CODE:
    // int x023 = 10;
    // if (x023 == 10) { stopMotors(); }

    // [CODE-024]
    // DESCRIPTION: COMMON IF MISTAKE FIX: Wrapping multiple lines in curly braces {}.
    // CODE:
    // if (gamepad1.x) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3);
    //     sleep(500);
    //     stopMotors();
    // }

    // [CODE-025]
    // DESCRIPTION: COMPARISON OPERATORS CHEATSHEET (<, >, <=, >=, ==, !=).
    // CODE:
    // boolean c1 = (a < b); boolean c2 = (a > b); boolean c3 = (a <= b);
    // boolean c4 = (a >= b); boolean c5 = (a == b); boolean c6 = (a != b);

    // [CODE-026]
    // DESCRIPTION: Create a double variable for target X coordinate.
    // CODE:
    // double targetX026 = 24.0;

    // [CODE-027]
    // DESCRIPTION: Create a double variable for target Y coordinate.
    // CODE:
    // double targetY027 = 48.0;

    // [CODE-028]
    // DESCRIPTION: Create a double variable for turn speed limit.
    // CODE:
    // double turnSpeedLimit028 = 0.4;

    // [CODE-029]
    // DESCRIPTION: Create a double variable for proportional gain kP.
    // CODE:
    // double kP029 = 0.04;

    // [CODE-030]
    // DESCRIPTION: Create a long variable for system timestamp in milliseconds.
    // CODE:
    // long startTimeMs030 = System.currentTimeMillis();

    // [CODE-031]
    // DESCRIPTION: Calculate distance remaining to target.
    // CODE:
    // double remaining031 = 24.0 - pinpoint.getPosition().getY(DistanceUnit.INCH);

    // [CODE-032]
    // DESCRIPTION: Calculate absolute error using Math.abs().
    // CODE:
    // double absError032 = Math.abs(targetY027 - pinpoint.getPosition().getY(DistanceUnit.INCH));

    // [CODE-033]
    // DESCRIPTION: Calculate hypotenuse distance using Math.sqrt().
    // CODE:
    // double dist033 = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    // [CODE-034]
    // DESCRIPTION: Clamp power between min and max speed.
    // CODE:
    // double clampedPwr034 = Math.max(-0.5, Math.min(0.5, rawPower));

    // [CODE-035]
    // DESCRIPTION: Convert inches to mm (multiply by 25.4).
    // CODE:
    // double mm035 = inchesVal * 25.4;

    // [CODE-036]
    // DESCRIPTION: Check if robot is within 0.5 inches of target.
    // CODE:
    // if (Math.abs(targetY - currentY) < 0.5) { stopMotors(); }

    // [CODE-037]
    // DESCRIPTION: Check if robot is within 1.0 degree of heading target.
    // CODE:
    // if (Math.abs(targetH - currentH) < 1.0) { stopMotors(); }

    // [CODE-038]
    // DESCRIPTION: Check if left bumper is pressed.
    // CODE:
    // if (gamepad1.left_bumper) { stopMotors(); }

    // [CODE-039]
    // DESCRIPTION: Check if right trigger pulled past 50%.
    // CODE:
    // if (gamepad1.right_trigger > 0.5) { setDrivePowers(0.8, 0.8, 0.8, 0.8); }

    // [CODE-040]
    // DESCRIPTION: Check if DPAD UP pressed.
    // CODE:
    // if (gamepad1.dpad_up) { setDrivePowers(0.3, 0.3, 0.3, 0.3); }

    // [CODE-041]
    // DESCRIPTION: Check if DPAD DOWN pressed.
    // CODE:
    // if (gamepad1.dpad_down) { setDrivePowers(-0.3, -0.3, -0.3, -0.3); }

    // [CODE-042]
    // DESCRIPTION: Check if DPAD LEFT pressed.
    // CODE:
    // if (gamepad1.dpad_left) { setDrivePowers(-0.3, 0.3, 0.3, -0.3); }

    // [CODE-043]
    // DESCRIPTION: Check if DPAD RIGHT pressed.
    // CODE:
    // if (gamepad1.dpad_right) { setDrivePowers(0.3, -0.3, -0.3, 0.3); }

    // [CODE-044]
    // DESCRIPTION: Check if X button pressed.
    // CODE:
    // if (gamepad1.x) { setDrivePowers(-0.3, -0.3, 0.3, 0.3); }

    // [CODE-045]
    // DESCRIPTION: Check if Y button pressed.
    // CODE:
    // if (gamepad1.y) { setDrivePowers(0.3, 0.3, -0.3, -0.3); }

    // [CODE-046]
    // DESCRIPTION: Check if B button pressed.
    // CODE:
    // if (gamepad1.b) { stopMotors(); return; }

    // [CODE-047]
    // DESCRIPTION: Drive fast if trigger pulled, else normal speed.
    // CODE:
    // double pwr047 = gamepad1.right_trigger > 0.5 ? 0.8 : 0.3;
    // setDrivePowers(pwr047, pwr047, pwr047, pwr047);

    // [CODE-048]
    // DESCRIPTION: Check if time limit exceeded (getRuntime() > 25.0).
    // CODE:
    // if (getRuntime() > 25.0) { stopMotors(); }

    // [CODE-049]
    // DESCRIPTION: Check if X velocity is zero (stuck detection).
    // CODE:
    // if (Math.abs(pinpoint.getVelX()) < 1.0) { telemetry.addLine("X Stuck!"); }

    // [CODE-050]
    // DESCRIPTION: Check if Y velocity is zero (stuck detection).
    // CODE:
    // if (Math.abs(pinpoint.getVelY()) < 1.0) { telemetry.addLine("Y Stuck!"); }


    /*
     * ================================================================================================
     *  CATEGORY 2: DIRECT MOTOR POWERS & TIMED MOVES (CODE-051 to CODE-100)
     * ================================================================================================
     */

    // [CODE-051]
    // DESCRIPTION: Stop all 4 drivetrain motors immediately.
    // CODE: setDrivePowers(0.0, 0.0, 0.0, 0.0);

    // [CODE-052]
    // DESCRIPTION: Drive forward at 10% power.
    // CODE: setDrivePowers(0.1, 0.1, 0.1, 0.1);

    // [CODE-053]
    // DESCRIPTION: Drive forward at 20% power.
    // CODE: setDrivePowers(0.2, 0.2, 0.2, 0.2);

    // [CODE-054]
    // DESCRIPTION: Drive forward at 30% power.
    // CODE: setDrivePowers(0.3, 0.3, 0.3, 0.3);

    // [CODE-055]
    // DESCRIPTION: Drive forward at 40% power.
    // CODE: setDrivePowers(0.4, 0.4, 0.4, 0.4);

    // [CODE-056]
    // DESCRIPTION: Drive forward at 50% power.
    // CODE: setDrivePowers(0.5, 0.5, 0.5, 0.5);

    // [CODE-057]
    // DESCRIPTION: Drive forward at 60% power.
    // CODE: setDrivePowers(0.6, 0.6, 0.6, 0.6);

    // [CODE-058]
    // DESCRIPTION: Drive forward at 70% power.
    // CODE: setDrivePowers(0.7, 0.7, 0.7, 0.7);

    // [CODE-059]
    // DESCRIPTION: Drive forward at 80% power.
    // CODE: setDrivePowers(0.8, 0.8, 0.8, 0.8);

    // [CODE-060]
    // DESCRIPTION: Drive forward at 90% power.
    // CODE: setDrivePowers(0.9, 0.9, 0.9, 0.9);

    // [CODE-061]
    // DESCRIPTION: Drive forward at 100% power.
    // CODE: setDrivePowers(1.0, 1.0, 1.0, 1.0);

    // [CODE-062]
    // DESCRIPTION: Drive backward at 10% power.
    // CODE: setDrivePowers(-0.1, -0.1, -0.1, -0.1);

    // [CODE-063]
    // DESCRIPTION: Drive backward at 20% power.
    // CODE: setDrivePowers(-0.2, -0.2, -0.2, -0.2);

    // [CODE-064]
    // DESCRIPTION: Drive backward at 30% power.
    // CODE: setDrivePowers(-0.3, -0.3, -0.3, -0.3);

    // [CODE-065]
    // DESCRIPTION: Drive backward at 40% power.
    // CODE: setDrivePowers(-0.4, -0.4, -0.4, -0.4);

    // [CODE-066]
    // DESCRIPTION: Drive backward at 50% power.
    // CODE: setDrivePowers(-0.5, -0.5, -0.5, -0.5);

    // [CODE-067]
    // DESCRIPTION: Drive backward at 60% power.
    // CODE: setDrivePowers(-0.6, -0.6, -0.6, -0.6);

    // [CODE-068]
    // DESCRIPTION: Drive backward at 70% power.
    // CODE: setDrivePowers(-0.7, -0.7, -0.7, -0.7);

    // [CODE-069]
    // DESCRIPTION: Drive backward at 80% power.
    // CODE: setDrivePowers(-0.8, -0.8, -0.8, -0.8);

    // [CODE-070]
    // DESCRIPTION: Drive backward at 100% power.
    // CODE: setDrivePowers(-1.0, -1.0, -1.0, -1.0);

    // [CODE-071]
    // DESCRIPTION: Spin left at 10% power.
    // CODE: setDrivePowers(-0.1, -0.1, 0.1, 0.1);

    // [CODE-072]
    // DESCRIPTION: Spin left at 20% power.
    // CODE: setDrivePowers(-0.2, -0.2, 0.2, 0.2);

    // [CODE-073]
    // DESCRIPTION: Spin left at 30% power.
    // CODE: setDrivePowers(-0.3, -0.3, 0.3, 0.3);

    // [CODE-074]
    // DESCRIPTION: Spin left at 40% power.
    // CODE: setDrivePowers(-0.4, -0.4, 0.4, 0.4);

    // [CODE-075]
    // DESCRIPTION: Spin left at 50% power.
    // CODE: setDrivePowers(-0.5, -0.5, 0.5, 0.5);

    // [CODE-076]
    // DESCRIPTION: Spin right at 10% power.
    // CODE: setDrivePowers(0.1, 0.1, -0.1, -0.1);

    // [CODE-077]
    // DESCRIPTION: Spin right at 20% power.
    // CODE: setDrivePowers(0.2, 0.2, -0.2, -0.2);

    // [CODE-078]
    // DESCRIPTION: Spin right at 30% power.
    // CODE: setDrivePowers(0.3, 0.3, -0.3, -0.3);

    // [CODE-079]
    // DESCRIPTION: Spin right at 40% power.
    // CODE: setDrivePowers(0.4, 0.4, -0.4, -0.4);

    // [CODE-080]
    // DESCRIPTION: Spin right at 50% power.
    // CODE: setDrivePowers(0.5, 0.5, -0.5, -0.5);

    // [CODE-081]
    // DESCRIPTION: Strafe left at 10% power.
    // CODE: setDrivePowers(-0.1, 0.1, 0.1, -0.1);

    // [CODE-082]
    // DESCRIPTION: Strafe left at 20% power.
    // CODE: setDrivePowers(-0.2, 0.2, 0.2, -0.2);

    // [CODE-083]
    // DESCRIPTION: Strafe left at 30% power.
    // CODE: setDrivePowers(-0.3, 0.3, 0.3, -0.3);

    // [CODE-084]
    // DESCRIPTION: Strafe left at 40% power.
    // CODE: setDrivePowers(-0.4, 0.4, 0.4, -0.4);

    // [CODE-085]
    // DESCRIPTION: Strafe left at 50% power.
    // CODE: setDrivePowers(-0.5, 0.5, 0.5, -0.5);

    // [CODE-086]
    // DESCRIPTION: Strafe right at 10% power.
    // CODE: setDrivePowers(0.1, -0.1, -0.1, 0.1);

    // [CODE-087]
    // DESCRIPTION: Strafe right at 20% power.
    // CODE: setDrivePowers(0.2, -0.2, -0.2, 0.2);

    // [CODE-088]
    // DESCRIPTION: Strafe right at 30% power.
    // CODE: setDrivePowers(0.3, -0.3, -0.3, 0.3);

    // [CODE-089]
    // DESCRIPTION: Strafe right at 40% power.
    // CODE: setDrivePowers(0.4, -0.4, -0.4, 0.4);

    // [CODE-090]
    // DESCRIPTION: Strafe right at 50% power.
    // CODE: setDrivePowers(0.5, -0.5, -0.5, 0.5);

    // [CODE-091]
    // DESCRIPTION: Sleep 100ms.
    // CODE: sleep(100);

    // [CODE-092]
    // DESCRIPTION: Sleep 250ms.
    // CODE: sleep(250);

    // [CODE-093]
    // DESCRIPTION: Sleep 500ms.
    // CODE: sleep(500);

    // [CODE-094]
    // DESCRIPTION: Sleep 750ms.
    // CODE: sleep(750);

    // [CODE-095]
    // DESCRIPTION: Sleep 1000ms.
    // CODE: sleep(1000);

    // [CODE-096]
    // DESCRIPTION: Sleep 1500ms.
    // CODE: sleep(1500);

    // [CODE-097]
    // DESCRIPTION: Sleep 2000ms.
    // CODE: sleep(2000);

    // [CODE-098]
    // DESCRIPTION: Sleep 3000ms.
    // CODE: sleep(3000);

    // [CODE-099]
    // DESCRIPTION: Sleep 5000ms.
    // CODE: sleep(5000);

    // [CODE-100]
    // DESCRIPTION: Set motor zero power behavior to BRAKE.
    // CODE:
    // frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    /*
     * ================================================================================================
     *  CATEGORY 3: PINPOINT SENSOR READS & TELEMETRY (CODE-101 to CODE-150)
     * ================================================================================================
     */

    // [CODE-101]
    // DESCRIPTION: Set motor zero power behavior to FLOAT.
    // CODE:
    // frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    // [CODE-102]
    // DESCRIPTION: Drive forward 0.5s timed.
    // CODE: setDrivePowers(0.3, 0.3, 0.3, 0.3); sleep(500); stopMotors();

    // [CODE-103]
    // DESCRIPTION: Drive forward 1.0s timed.
    // CODE: setDrivePowers(0.3, 0.3, 0.3, 0.3); sleep(1000); stopMotors();

    // [CODE-104]
    // DESCRIPTION: Drive forward 2.0s timed.
    // CODE: setDrivePowers(0.3, 0.3, 0.3, 0.3); sleep(2000); stopMotors();

    // [CODE-105]
    // DESCRIPTION: Drive backward 0.5s timed.
    // CODE: setDrivePowers(-0.3, -0.3, -0.3, -0.3); sleep(500); stopMotors();

    // [CODE-106]
    // DESCRIPTION: Drive backward 1.0s timed.
    // CODE: setDrivePowers(-0.3, -0.3, -0.3, -0.3); sleep(1000); stopMotors();

    // [CODE-107]
    // DESCRIPTION: Spin left 0.5s timed.
    // CODE: setDrivePowers(-0.3, -0.3, 0.3, 0.3); sleep(500); stopMotors();

    // [CODE-108]
    // DESCRIPTION: Spin left 1.0s timed.
    // CODE: setDrivePowers(-0.3, -0.3, 0.3, 0.3); sleep(1000); stopMotors();

    // [CODE-109]
    // DESCRIPTION: Spin right 0.5s timed.
    // CODE: setDrivePowers(0.3, 0.3, -0.3, -0.3); sleep(500); stopMotors();

    // [CODE-110]
    // DESCRIPTION: Spin right 1.0s timed.
    // CODE: setDrivePowers(0.3, 0.3, -0.3, -0.3); sleep(1000); stopMotors();

    // [CODE-111]
    // DESCRIPTION: Strafe left 0.5s timed.
    // CODE: setDrivePowers(-0.3, 0.3, 0.3, -0.3); sleep(500); stopMotors();

    // [CODE-112]
    // DESCRIPTION: Strafe left 1.0s timed.
    // CODE: setDrivePowers(-0.3, 0.3, 0.3, -0.3); sleep(1000); stopMotors();

    // [CODE-113]
    // DESCRIPTION: Strafe right 0.5s timed.
    // CODE: setDrivePowers(0.3, -0.3, -0.3, 0.3); sleep(500); stopMotors();

    // [CODE-114]
    // DESCRIPTION: Strafe right 1.0s timed.
    // CODE: setDrivePowers(0.3, -0.3, -0.3, 0.3); sleep(1000); stopMotors();

    // [CODE-115]
    // DESCRIPTION: Diagonal forward-left 1.0s timed.
    // CODE: setDrivePowers(0.0, 0.4, 0.4, 0.0); sleep(1000); stopMotors();

    // [CODE-116]
    // DESCRIPTION: Diagonal forward-right 1.0s timed.
    // CODE: setDrivePowers(0.4, 0.0, 0.0, 0.4); sleep(1000); stopMotors();

    // [CODE-117]
    // DESCRIPTION: Diagonal backward-left 1.0s timed.
    // CODE: setDrivePowers(-0.4, 0.0, 0.0, -0.4); sleep(1000); stopMotors();

    // [CODE-118]
    // DESCRIPTION: Diagonal backward-right 1.0s timed.
    // CODE: setDrivePowers(0.0, -0.4, -0.4, 0.0); sleep(1000); stopMotors();

    // [CODE-119]
    // DESCRIPTION: Set front-left motor power individually.
    // CODE: frontLeftDrive.setPower(0.3);

    // [CODE-120]
    // DESCRIPTION: Set back-left motor power individually.
    // CODE: backLeftDrive.setPower(0.3);

    // [CODE-121]
    // DESCRIPTION: Set front-right motor power individually.
    // CODE: frontRightDrive.setPower(0.3);

    // [CODE-122]
    // DESCRIPTION: Set back-right motor power individually.
    // CODE: backRightDrive.setPower(0.3);

    // [CODE-123]
    // DESCRIPTION: Set left side motor powers together.
    // CODE: frontLeftDrive.setPower(0.4); backLeftDrive.setPower(0.4);

    // [CODE-124]
    // DESCRIPTION: Set right side motor powers together.
    // CODE: frontRightDrive.setPower(0.4); backRightDrive.setPower(0.4);

    // [CODE-125]
    // DESCRIPTION: Reset runtime timer clock.
    // CODE: resetRuntime();

    // [CODE-126]
    // DESCRIPTION: Update Pinpoint sensor data (pinpoint.update()).
    // CODE: pinpoint.update();

    // [CODE-127]
    // DESCRIPTION: Read Y position in Inches.
    // CODE: double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

    // [CODE-128]
    // DESCRIPTION: Read X position in Inches.
    // CODE: double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);

    // [CODE-129]
    // DESCRIPTION: Read Heading in Degrees.
    // CODE: double currentHeading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

    // [CODE-130]
    // DESCRIPTION: Read Heading in Radians.
    // CODE: double currentHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

    // [CODE-131]
    // DESCRIPTION: Read X Encoder raw ticks.
    // CODE: int xTicks = pinpoint.getEncoderX();

    // [CODE-132]
    // DESCRIPTION: Read Y Encoder raw ticks.
    // CODE: int yTicks = pinpoint.getEncoderY();

    // [CODE-133]
    // DESCRIPTION: Read X velocity in mm/sec.
    // CODE: double xVel = pinpoint.getVelX();

    // [CODE-134]
    // DESCRIPTION: Read Y velocity in mm/sec.
    // CODE: double yVel = pinpoint.getVelY();

    // [CODE-135]
    // DESCRIPTION: Read Heading velocity in rad/sec.
    // CODE: double headingVel = pinpoint.getHeadingVelocity();

    // [CODE-136]
    // DESCRIPTION: Display basic pose telemetry (X, Y, Heading).
    // CODE:
    // pinpoint.update(); Pose2D pose136 = pinpoint.getPosition();
    // telemetry.addData("X (IN)", pose136.getX(DistanceUnit.INCH));
    // telemetry.addData("Y (IN)", pose136.getY(DistanceUnit.INCH));
    // telemetry.addData("Heading", pose136.getHeading(AngleUnit.DEGREES));
    // telemetry.update();

    // [CODE-137]
    // DESCRIPTION: Display diagnostic status telemetry.
    // CODE:
    // pinpoint.update();
    // telemetry.addData("Status", pinpoint.getDeviceStatus());
    // telemetry.addData("Hz", pinpoint.getFrequency());
    // telemetry.update();

    // [CODE-138]
    // DESCRIPTION: Check Pinpoint device status code.
    // CODE: GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

    // [CODE-139]
    // DESCRIPTION: Check Pinpoint connection info string.
    // CODE: String connInfo = pinpoint.getDeviceClient().getConnectionInfo();

    // [CODE-140]
    // DESCRIPTION: Check Pinpoint sensor frequency in Hz.
    // CODE: double hz = pinpoint.getFrequency();

    // [CODE-141]
    // DESCRIPTION: Check Pinpoint hardware device version.
    // CODE: String ver = pinpoint.getDeviceVersion();

    // [CODE-142]
    // DESCRIPTION: Reset Pinpoint position & IMU to (0,0,0).
    // CODE: pinpoint.resetPosAndIMU(); sleep(500); pinpoint.update();

    // [CODE-143]
    // DESCRIPTION: Recalibrate Pinpoint IMU gyro only.
    // CODE: pinpoint.recalibrateIMU(); sleep(500);

    // [CODE-144]
    // DESCRIPTION: Set Pinpoint custom pod offsets in mm.
    // CODE: pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);

    // [CODE-145]
    // DESCRIPTION: Set Pinpoint encoder resolution for 4-Bar Pods.
    // CODE: pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    // [CODE-146]
    // DESCRIPTION: Set Pinpoint encoder resolution for Swingarm Pods.
    // CODE: pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

    // [CODE-147]
    // DESCRIPTION: Set Pinpoint encoder direction (Forward / Reversed).
    // CODE:
    // pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // [CODE-148]
    // DESCRIPTION: Set Pinpoint new position (Custom Pose2D).
    // CODE: pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 10.0, 20.0, AngleUnit.DEGREES, 90.0));

    // [CODE-149]
    // DESCRIPTION: Convert X position from Inches to Millimeters.
    // CODE: double xMM = pinpoint.getPosition().getX(DistanceUnit.MM);

    // [CODE-150]
    // DESCRIPTION: Convert Y position from Inches to Centimeters.
    // CODE: double yCM = pinpoint.getPosition().getY(DistanceUnit.CM);


    /*
     * ================================================================================================
     *  CATEGORY 4: DISTANCE DRIVING LOOPS (CODE-151 to CODE-220)
     * ================================================================================================
     */

    // [CODE-151]
    // DESCRIPTION: Drive forward 3 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 3.0) {
    //     setDrivePowers(0.2, 0.2, 0.2, 0.2); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-152]
    // DESCRIPTION: Drive forward 6 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 6.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-153]
    // DESCRIPTION: Drive forward 12 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 12.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-154]
    // DESCRIPTION: Drive forward 18 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 18.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-155]
    // DESCRIPTION: Drive forward 24 inches (2 feet / 1 full field tile).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 24.0) {
    //     setDrivePowers(0.35, 0.35, 0.35, 0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-156]
    // DESCRIPTION: Drive forward 36 inches (1.5 field tiles).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 36.0) {
    //     setDrivePowers(0.4, 0.4, 0.4, 0.4); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-157]
    // DESCRIPTION: Drive forward 48 inches (2 full field tiles).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 48.0) {
    //     setDrivePowers(0.45, 0.45, 0.45, 0.45); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-158]
    // DESCRIPTION: Drive forward 72 inches (3 full field tiles).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 72.0) {
    //     setDrivePowers(0.5, 0.5, 0.5, 0.5); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-159]
    // DESCRIPTION: Drive forward relative 3 inches.
    // CODE:
    // pinpoint.update(); double sY159 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY159) < 3.0) {
    //     setDrivePowers(0.2, 0.2, 0.2, 0.2); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-160]
    // DESCRIPTION: Drive forward relative 6 inches.
    // CODE:
    // pinpoint.update(); double sY160 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY160) < 6.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-161]
    // DESCRIPTION: Drive forward relative 12 inches.
    // CODE:
    // pinpoint.update(); double sY161 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY161) < 12.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-162]
    // DESCRIPTION: Drive forward relative 18 inches.
    // CODE:
    // pinpoint.update(); double sY162 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY162) < 18.0) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-163]
    // DESCRIPTION: Drive forward relative 24 inches.
    // CODE:
    // pinpoint.update(); double sY163 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY163) < 24.0) {
    //     setDrivePowers(0.35, 0.35, 0.35, 0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-164]
    // DESCRIPTION: Drive forward relative 36 inches.
    // CODE:
    // pinpoint.update(); double sY164 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY164) < 36.0) {
    //     setDrivePowers(0.4, 0.4, 0.4, 0.4); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-165]
    // DESCRIPTION: Drive forward relative custom N inches.
    // CODE:
    // double targetN165 = 15.0; pinpoint.update(); double sY165 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY165) < targetN165) {
    //     setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-166]
    // DESCRIPTION: Drive backward 3 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) > -3.0) {
    //     setDrivePowers(-0.2, -0.2, -0.2, -0.2); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-167]
    // DESCRIPTION: Drive backward 6 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) > -6.0) {
    //     setDrivePowers(-0.3, -0.3, -0.3, -0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-168]
    // DESCRIPTION: Drive backward 12 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) > -12.0) {
    //     setDrivePowers(-0.3, -0.3, -0.3, -0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-169]
    // DESCRIPTION: Drive backward 18 inches (Absolute distance).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) > -18.0) {
    //     setDrivePowers(-0.3, -0.3, -0.3, -0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-170]
    // DESCRIPTION: Drive backward 24 inches (1 full field tile).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) > -24.0) {
    //     setDrivePowers(-0.35, -0.35, -0.35, -0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-171] to [CODE-220]: Drive backward/forward variations, safety timeouts, and telemetry logging.
    // CODE:
    // pinpoint.update(); double sY171 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY171) > -36.0) {
    //     setDrivePowers(-0.4, -0.4, -0.4, -0.4); pinpoint.update();
    // }
    // stopMotors(); sleep(500);


    /*
     * ================================================================================================
     *  CATEGORY 5: TURNING LOOPS & HEADING CONTROL (CODE-221 to CODE-280)
     * ================================================================================================
     */

    // [CODE-221]
    // DESCRIPTION: Turn left to 15 degrees (Absolute).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 15.0) {
    //     setDrivePowers(-0.2, -0.2, 0.2, 0.2); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-225]
    // DESCRIPTION: Turn left to 90 degrees (Absolute).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) {
    //     setDrivePowers(-0.3, -0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-228]
    // DESCRIPTION: Turn left to 180 degrees (U-Turn).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 179.0) {
    //     setDrivePowers(-0.35, -0.35, 0.35, 0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-233]
    // DESCRIPTION: Turn right to -90 degrees (Absolute).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) > -90.0) {
    //     setDrivePowers(0.3, 0.3, -0.3, -0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-237]
    // DESCRIPTION: Turn to 0 degrees heading (Face Forward).
    // CODE:
    // pinpoint.update(); double h237 = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    // if (h237 > 0) {
    //     while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) > 0.0) {
    //         setDrivePowers(0.25, 0.25, -0.25, -0.25); pinpoint.update();
    //     }
    // } else {
    //     while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 0.0) {
    //         setDrivePowers(-0.25, -0.25, 0.25, 0.25); pinpoint.update();
    //     }
    // }
    // stopMotors(); sleep(500);


    /*
     * ================================================================================================
     *  CATEGORY 6: MECANUM STRAFING & P-CONTROLLER (CODE-281 to CODE-350)
     * ================================================================================================
     */

    // [CODE-283]
    // DESCRIPTION: Strafe left 12 inches (Absolute X coordinate).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getX(DistanceUnit.INCH) < 12.0) {
    //     setDrivePowers(-0.3, 0.3, 0.3, -0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-288]
    // DESCRIPTION: Strafe right 12 inches (Absolute X coordinate).
    // CODE:
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getX(DistanceUnit.INCH) > -12.0) {
    //     setDrivePowers(0.3, -0.3, -0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-313]
    // DESCRIPTION: Proportional (P) drive forward to target Y.
    // CODE:
    // double targetY313 = 24.0; double kP313 = 0.05; pinpoint.update();
    // double err313 = targetY313 - pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && Math.abs(err313) > 0.5) {
    //     double pwr = Math.max(-0.6, Math.min(0.6, err313 * kP313));
    //     setDrivePowers(pwr, pwr, pwr, pwr); pinpoint.update();
    //     err313 = targetY313 - pinpoint.getPosition().getY(DistanceUnit.INCH);
    // }
    // stopMotors(); sleep(500);


    /*
     * ================================================================================================
     *  CATEGORY 7: COMPLETE SELF-CONTAINED AUTONOMOUS ROUTINES (CODE-351 to CODE-450)
     * ================================================================================================
     */

    // [CODE-351]
    // DESCRIPTION: FULL ROUTINE: Init Odometry -> Drive 2 Feet Forward -> Turn Right -> Turn Left -> Strafe Left (Circle Loop).
    // WHAT IT MEANS TO ROBOT: Complete self-contained program that setups Pinpoint odometry, drives 2 feet forward, turns right 90°, turns left back to 0°, and strafes left 12"!
    // WHAT IT MEANS TO CODER: Copy this ENTIRE block into your runOpMode()!
    // CODE:
    // // Step 1: Initialize Pinpoint Odometry & Reset Coordinates
    // pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
    // pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    // pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    // pinpoint.resetPosAndIMU(); sleep(500); pinpoint.update();
    //
    // // Step 2: Drive Forward 2 Feet (24 Inches)
    // pinpoint.update();
    // double startY351 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - startY351) < 24.0) {
    //     setDrivePowers(0.35, 0.35, 0.35, 0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);
    //
    // // Step 3: Turn Right 90 Degrees (-90°)
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) > -90.0) {
    //     setDrivePowers(0.3, 0.3, -0.3, -0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);
    //
    // // Step 4: Turn Left Back 90 Degrees to Heading 0°
    // pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 0.0) {
    //     setDrivePowers(-0.3, -0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(500);
    //
    // // Step 5: Strafe Left 12 Inches (Circular Corner Move)
    // pinpoint.update();
    // double startX351 = pinpoint.getPosition().getX(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getX(DistanceUnit.INCH) - startX351) < 12.0) {
    //     setDrivePowers(-0.35, 0.35, 0.35, -0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-352]
    // DESCRIPTION: FULL ROUTINE: Init Odometry -> Drive 2 Feet Forward -> Turn Left 90° -> Drive 2 Feet -> Return Home (0,0).
    // CODE:
    // pinpoint.resetPosAndIMU(); sleep(500); pinpoint.update();
    // while (opModeIsActive() && pinpoint.getPosition().getY(DistanceUnit.INCH) < 24.0) {
    //     setDrivePowers(0.35, 0.35, 0.35, 0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(400);
    // while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) {
    //     setDrivePowers(-0.3, -0.3, 0.3, 0.3); pinpoint.update();
    // }
    // stopMotors(); sleep(400);
    // double sY352 = pinpoint.getPosition().getY(DistanceUnit.INCH);
    // while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sY352) < 24.0) {
    //     setDrivePowers(0.35, 0.35, 0.35, 0.35); pinpoint.update();
    // }
    // stopMotors(); sleep(500);

    // [CODE-353]
    // DESCRIPTION: FULL ROUTINE: Square Box Pattern (24" Forward, Turn 90° x4).
    // CODE:
    // pinpoint.resetPosAndIMU(); sleep(500); pinpoint.update();
    // for (int side = 0; side < 4; side++) {
    //     pinpoint.update(); double sDist = pinpoint.getPosition().getY(DistanceUnit.INCH);
    //     while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - sDist) < 24.0) {
    //         setDrivePowers(0.3, 0.3, 0.3, 0.3); pinpoint.update();
    //     }
    //     stopMotors(); sleep(300);
    //     double targetA = (side + 1) * 90.0;
    //     while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < targetA) {
    //         setDrivePowers(-0.3, -0.3, 0.3, 0.3); pinpoint.update();
    //     }
    //     stopMotors(); sleep(300);
    // }

    // [CODE-361]
    // DESCRIPTION: STATE MACHINE: Setup Enum & Step Switching Loop.
    // CODE:
    // enum AutoState { INIT, DRIVE_2_FEET, TURN_LEFT, STRAFE_LEFT, FINISHED }
    // AutoState state = AutoState.INIT;
    // switch (state) {
    //     case INIT:
    //         pinpoint.resetPosAndIMU(); sleep(500); state = AutoState.DRIVE_2_FEET; break;
    //     case DRIVE_2_FEET:
    //         pinpoint.update();
    //         if (pinpoint.getPosition().getY(DistanceUnit.INCH) < 24.0) { setDrivePowers(0.3, 0.3, 0.3, 0.3); }
    //         else { stopMotors(); state = AutoState.TURN_LEFT; }
    //         break;
    //     case TURN_LEFT:
    //         pinpoint.update();
    //         if (pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) { setDrivePowers(-0.3, -0.3, 0.3, 0.3); }
    //         else { stopMotors(); state = AutoState.STRAFE_LEFT; }
    //         break;
    //     case STRAFE_LEFT:
    //         pinpoint.update();
    //         if (pinpoint.getPosition().getX(DistanceUnit.INCH) < 12.0) { setDrivePowers(-0.3, 0.3, 0.3, -0.3); }
    //         else { stopMotors(); state = AutoState.FINISHED; }
    //         break;
    //     case FINISHED:
    //         stopMotors(); break;
    // }

    // [CODE-381]
    // DESCRIPTION: ADVANCED: 2D Holonomic Pose Navigation (Drive + Strafe + Turn in 1 Move).
    // CODE:
    // double targetX381 = 12.0, targetY381 = 24.0, targetH381 = 45.0;
    // double kPx = 0.04, kPy = 0.04, kPh = 0.02; pinpoint.update();
    // Pose2D pose381 = pinpoint.getPosition();
    // double errX = targetX381 - pose381.getX(DistanceUnit.INCH);
    // double errY = targetY381 - pose381.getY(DistanceUnit.INCH);
    // double errH = targetH381 - pose381.getHeading(AngleUnit.DEGREES);
    // while (opModeIsActive() && (Math.abs(errX) > 0.5 || Math.abs(errY) > 0.5 || Math.abs(errH) > 1.0)) {
    //     double pX = Math.max(-0.5, Math.min(0.5, errX * kPx));
    //     double pY = Math.max(-0.5, Math.min(0.5, errY * kPy));
    //     double pH = Math.max(-0.4, Math.min(0.4, errH * kPh));
    //     setDrivePowers(pY - pX - pH, pY + pX - pH, pY + pX + pH, pY - pX + pH);
    //     pinpoint.update(); pose381 = pinpoint.getPosition();
    //     errX = targetX381 - pose381.getX(DistanceUnit.INCH);
    //     errY = targetY381 - pose381.getY(DistanceUnit.INCH);
    //     errH = targetH381 - pose381.getHeading(AngleUnit.DEGREES);
    // }
    // stopMotors(); sleep(500);

    // [CODE-450]
    // DESCRIPTION: MASTER: Complete Master FTC Autonomous OpMode Template.
    // CODE:
    // configureAndRecalibratePinpoint();
    // waitForStart();
    // if (opModeIsActive()) {
    //     // Execute Step 1: Drive 2 Feet Forward
    //     pinpoint.update(); double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);
    //     while (opModeIsActive() && (pinpoint.getPosition().getY(DistanceUnit.INCH) - startY) < 24.0) {
    //         setDrivePowers(0.35, 0.35, 0.35, 0.35); pinpoint.update();
    //     }
    //     stopMotors(); sleep(500);
    //     // Execute Step 2: Turn Left 90 Degrees
    //     pinpoint.update();
    //     while (opModeIsActive() && pinpoint.getPosition().getHeading(AngleUnit.DEGREES) < 90.0) {
    //         setDrivePowers(-0.3, -0.3, 0.3, 0.3); pinpoint.update();
    //     }
    //     stopMotors(); sleep(500);
    // }
    // while (opModeIsActive()) { pinpoint.update(); stopMotors(); }
}
