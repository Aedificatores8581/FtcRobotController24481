package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@Config
@Autonomous(name = "Telemetry Test", group = "Test")
public class TelemetryTestAuto extends LinearOpMode {

    private static final String TAG = "TelemetryTest";

    // change these live on FTC Dashboard under TelemetryTestAuto
    public static double RUN_SECONDS = 20;
    public static double LOOP_MS = 100;
    public static boolean SEND_PACKET = true;   // field + graph packet on top of normal telemetry
    public static double FIELD_RADIUS_IN = 24;  // circle the dot travels on

    private PrintWriter logFile;
    private String logPath = "none";

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // DS + Dashboard. This is the only wrapper in this OpMode: everything below uses `telemetry`.
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        openLogFile();
        log("init", "dashboard=" + (dashboard != null) + " logFile=" + logPath);

        telemetry.addData("Status", "Ready. Press play.");
        telemetry.addData("Dashboard", dashboard != null ? "connected object ok" : "NULL");
        telemetry.addData("Log file", logPath);
        telemetry.addData("CLI", "adb logcat -s " + TAG);
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (opModeIsActive() && timer.seconds() < RUN_SECONDS) {
            double t = timer.seconds();
            double sine = Math.sin(t);
            count++;

            // 1 + 2: Driver Station and Dashboard telemetry pane (numbers also graph on Dashboard)
            telemetry.addData("Test Telemetry", "alive");
            telemetry.addData("time", "%.1f / %.0f s", t, RUN_SECONDS);
            telemetry.addData("count", count);
            telemetry.addData("sine", sine);
            telemetry.addData("packet", SEND_PACKET ? "on" : "off");
            telemetry.update();

            // Dashboard packet: field overlay. Shows what packets can do that telemetry can't.
            if (SEND_PACKET) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("packet count", count);
                double x = FIELD_RADIUS_IN * Math.cos(t);
                double y = FIELD_RADIUS_IN * Math.sin(t);
                Canvas field = packet.fieldOverlay();
                field.setStroke("#3F51B5");
                field.strokeCircle(0, 0, FIELD_RADIUS_IN);
                field.setFill("#4CAF50");
                field.fillCircle(x, y, 3);
                dashboard.sendTelemetryPacket(packet);
            }

            // 3 + 4: CLI (logcat) and local file
            log("loop", String.format(Locale.US, "t=%.2f count=%d sine=%.3f", t, count, sine));

            sleep((long) LOOP_MS);
        }

        log("done", "loops=" + count);
        closeLogFile();

        telemetry.addData("Status", "Done. loops=" + count);
        telemetry.update();
        // keep the final screen up briefly so it can be read
        sleep(2000);
    }

    /** One call writes to logcat (CLI) and the local log file. */
    private void log(String stage, String msg) {
        String line = stage + ": " + msg;
        RobotLog.ii(TAG, line);
        if (logFile != null) {
            logFile.println(new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date()) + " " + line);
            logFile.flush(); // flush every line so a crash or e-stop doesn't lose the end of the log
        }
    }

    private void openLogFile() {
        try {
            File dir = new File(AppUtil.ROBOT_DATA_DIR, "telemetry");
            if (!dir.exists() && !dir.mkdirs()) {
                throw new IOException("could not create " + dir);
            }
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            File file = new File(dir, "telemetry_" + stamp + ".log");
            logFile = new PrintWriter(new FileWriter(file, true));
            logPath = file.getAbsolutePath();
        } catch (IOException e) {
            logFile = null;
            logPath = "FAILED: " + e.getMessage();
            RobotLog.ee(TAG, e, "could not open log file");
        }
    }

    private void closeLogFile() {
        if (logFile != null) {
            logFile.close();
            logFile = null;
        }
    }
}
