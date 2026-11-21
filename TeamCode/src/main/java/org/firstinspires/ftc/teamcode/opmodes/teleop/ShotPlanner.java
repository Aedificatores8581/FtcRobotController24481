package org.firstinspires.ftc.teamcode.opmodes.teleop;

/**
 * Where to shoot from, and how to get there around the hive. Pure math, no hardware.
 *
 * Field frame (meters): origin = field center, +X / +Y as in the field model
 * (red wall at -X, blue side at +Y). Heading is CCW from +X.
 * Cluster numbers: 0 red scoring, 1 red audience, 2 blue audience, 3 blue scoring.
 */
public final class ShotPlanner {

    private ShotPlanner() {}

    /** {plate center x, y, z (upright), side the shooter stands on: -1 = -X side, +1 = +X side} */
    public static final double[][] PLATE = {
            {-0.2894, -0.3236, 0.9054, -1},   // 0 red scoring
            { 0.3273, -0.3236, 1.2615, +1},   // 1 red audience
            {-0.3273,  0.3241, 1.2615, -1},   // 2 blue audience
            { 0.2894,  0.3241, 0.9054, +1},   // 3 blue scoring
    };

    public static final double FIELD_HALF = 1.82;
    public static final double HIVE_X = 0.64;
    public static final double HIVE_Y = 0.62;
    /** Robot may rotate, so keep a full corner radius (18 in square) away from the hive. */
    public static final double HIVE_CLEARANCE = 0.34;

    public static int scoringCluster(int alliance) { return alliance == 1 ? 0 : 3; }
    public static int audienceCluster(int alliance) { return alliance == 1 ? 1 : 2; }
    public static boolean isScoring(int cluster) { return cluster == 0 || cluster == 3; }

    /** Horizontal camera-to-plate-center distance at which the camera reads ty = tyDeg. */
    public static double shotDistance(double plateZ, double camZ, double pitchDeg, double tyDeg) {
        double elev = Math.toRadians(pitchDeg + tyDeg);
        return (plateZ - camZ) / Math.tan(elev);
    }

    /** Audience-plate ty that gives the same shooting distance as the scoring plate at scoringTy. */
    public static double audienceTy(double camZ, double pitchDeg, double scoringTy) {
        double d = shotDistance(PLATE[0][2], camZ, pitchDeg, scoringTy);
        return Math.toDegrees(Math.atan((PLATE[1][2] - camZ) / d)) - pitchDeg;
    }

    /** Camera pitch implied by a ty reading from a known horizontal distance to a plate. */
    public static double impliedPitch(int cluster, double camZ, double horizDist, double tyDeg) {
        return Math.toDegrees(Math.atan((PLATE[cluster][2] - camZ) / horizDist)) - tyDeg;
    }

    /** {robot x, robot y, heading rad}: directly in front of the plate, camera dist away, facing it. */
    public static double[] goal(int cluster, double dist, double camFwd, double robotHalfLen) {
        double side = PLATE[cluster][3];
        double h = side < 0 ? 0.0 : Math.PI;
        double camX = PLATE[cluster][0] + side * dist;
        double camY = PLATE[cluster][1];
        double rx = camX - camFwd * Math.cos(h);
        double ry = camY - camFwd * Math.sin(h);
        double lim = FIELD_HALF - robotHalfLen - 0.02;
        rx = Math.max(-lim, Math.min(lim, rx));
        ry = Math.max(-lim, Math.min(lim, ry));
        return new double[]{rx, ry, h};
    }

    // ---------------- path around the hive (visibility graph, 6 nodes) ----------------

    private static boolean blocked(double ax, double ay, double bx, double by) {
        double hx = HIVE_X + HIVE_CLEARANCE - 0.04, hy = HIVE_Y + HIVE_CLEARANCE - 0.04;
        double t0 = 0, t1 = 1;
        double[] d = {bx - ax, by - ay};
        double[] o = {ax, ay};
        double[] h = {hx, hy};
        for (int i = 0; i < 2; i++) {
            if (Math.abs(d[i]) < 1e-9) {
                if (Math.abs(o[i]) >= h[i]) return false;
            } else {
                double ta = (-h[i] - o[i]) / d[i], tb = (h[i] - o[i]) / d[i];
                if (ta > tb) { double s = ta; ta = tb; tb = s; }
                t0 = Math.max(t0, ta);
                t1 = Math.min(t1, tb);
                if (t0 >= t1) return false;
            }
        }
        return t1 > t0 + 1e-6;
    }

    /** Waypoints from start to goal (both included), going around the hive when needed. */
    public static double[][] path(double sx, double sy, double gx, double gy) {
        double cx = HIVE_X + HIVE_CLEARANCE, cy = HIVE_Y + HIVE_CLEARANCE;
        double[][] n = {{sx, sy}, {gx, gy}, {cx, cy}, {-cx, cy}, {-cx, -cy}, {cx, -cy}};
        int N = n.length;
        double[] dist = new double[N];
        int[] prev = new int[N];
        boolean[] done = new boolean[N];
        java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(prev, -1);
        dist[0] = 0;
        for (int it = 0; it < N; it++) {
            int u = -1;
            for (int i = 0; i < N; i++) if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
            if (u < 0 || Double.isInfinite(dist[u])) break;
            done[u] = true;
            for (int v = 0; v < N; v++) {
                if (done[v] || v == u) continue;
                if (blocked(n[u][0], n[u][1], n[v][0], n[v][1])) continue;
                double nd = dist[u] + Math.hypot(n[u][0] - n[v][0], n[u][1] - n[v][1]);
                if (nd < dist[v]) { dist[v] = nd; prev[v] = u; }
            }
        }
        if (Double.isInfinite(dist[1])) return new double[][]{{sx, sy}, {gx, gy}};
        java.util.ArrayList<double[]> out = new java.util.ArrayList<>();
        for (int v = 1; v != -1; v = prev[v]) out.add(n[v]);
        java.util.Collections.reverse(out);
        return out.toArray(new double[0][]);
    }

    /**
     * Drive command toward the next waypoint, in ROBOT terms.
     * Returns {axial, lateral(+ = strafe right), yaw(+ = turn clockwise), distanceToGoal}.
     * Heading control faces the plate center. Mecanum sign convention matches MainTeleOpBase.
     */
    public static double[] drive(double x, double y, double h, double angVel,
                                 double[][] path, double plateX, double plateY,
                                 double maxPower, double minPower, double posGain,
                                 double yawGain, double maxYaw, double velDamp, double tol) {
        int wi = 1;
        while (wi < path.length - 1 && Math.hypot(path[wi][0] - x, path[wi][1] - y) < 0.12) wi++;
        double[] gp = path[path.length - 1];
        double toGoal = Math.hypot(gp[0] - x, gp[1] - y);
        double e = Math.atan2(plateY - y, plateX - x) - h;
        while (e > Math.PI) e -= 2 * Math.PI;
        while (e < -Math.PI) e += 2 * Math.PI;
        double yaw = -e * yawGain + angVel * velDamp;
        yaw = Math.max(-maxYaw, Math.min(maxYaw, yaw));
        if (toGoal < tol) return new double[]{0, 0, yaw, toGoal};
        double[] wp = path[wi];
        double dx = wp[0] - x, dy = wp[1] - y, d = Math.hypot(dx, dy);
        boolean last = wi == path.length - 1;
        double p = last ? Math.max(minPower, Math.min(maxPower, posGain * d)) : maxPower;
        double vx = dx / d * p, vy = dy / d * p;
        double fwd = vx * Math.cos(h) + vy * Math.sin(h);
        double left = -vx * Math.sin(h) + vy * Math.cos(h);
        return new double[]{fwd, -left, yaw, toGoal};
    }
}
