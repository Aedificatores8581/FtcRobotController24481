# FTC Dashboard Step-by-Step Setup Guide

This guide provides a simple, step-by-step process for configuring and running **FTC Dashboard** (`com.acmerobotics.dashboard:dashboard:0.6.0`) on your FTC robot.

---

## Step 1: Verify Dependencies in Gradle (Already Completed)

1. Open `build.dependencies.gradle` in your project root.
2. Ensure the `https://maven.brott.dev/` repository is added:
   ```groovy
   repositories {
       mavenCentral()
       google()
       maven { url = 'https://maven.brott.dev/' }
   }
   ```
3. Ensure the FTC Dashboard dependency is listed:
   ```groovy
   dependencies {
       ...
       implementation 'com.acmerobotics.dashboard:dashboard:0.6.0'
   }
   ```
4. Click **Sync Project with Gradle Files** in Android Studio.

---

## Step 2: Add FTC Dashboard Telemetry to Your OpMode

1. Add the following imports at the top of your OpMode Java file:
   ```java
   import com.acmerobotics.dashboard.FtcDashboard;
   import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
   ```

2. Inside `runOpMode()`, initialize `MultipleTelemetry` at the beginning of setup:
   ```java
   @Override
   public void runOpMode() {
       // Stream telemetry to both Driver Station phone and FTC Dashboard browser
       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
       
       ...
   }
   ```

---

## Step 3: Connect Computer to Robot Wi-Fi

1. Turn on the REV Control Hub on your robot.
2. On your computer, open Wi-Fi settings and connect to the Control Hub's Wi-Fi network (e.g., `12345-RC`).
3. Enter the Wi-Fi password (found in the Driver Station *Program & Manage* menu).

---

## Step 4: Deploy App & Initialize OpMode

1. In Android Studio, select `TeamCode` and click the green **Run (Play)** button to deploy your code to the robot.
2. On the Driver Station phone, select your OpMode (e.g., `goBILDA® Pinpoint Odometry Computer v2`) and press **INIT**.

---

## Step 5: Open FTC Dashboard in Computer Web Browser

1. Open Chrome, Edge, or Safari on your computer.
2. Type the following URL into your browser address bar:

   👉 **`http://192.168.43.1:8080/dash/`**

> **CRITICAL TIP**: You **MUST** include the trailing slash `/` at the very end (`/dash/`).
> If you leave off the trailing slash and type `192.168.43.1:8080/dash`, the server will return `Mime type unknown: uri='/dash' path='dash'`.

---

## Step 6: Using FTC Dashboard Features

* **Telemetry Panel**: View live numerical outputs (`Forward X`, `Strafe Y`, `Heading`, `Device Status`).
* **Plotter Panel**: View real-time graphs of position or speed over time.
* **Field View**: View 2D field overlays showing robot pose on the FTC field canvas.
* **Config Panel**: Live tune variables without re-deploying code.
