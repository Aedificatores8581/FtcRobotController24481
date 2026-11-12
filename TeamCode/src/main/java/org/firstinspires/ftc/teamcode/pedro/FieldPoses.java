package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

// Field positions in inches and degrees.
// Origin is the audience-left corner, +X toward the blue wall, +Y toward the far wall.
// Heading 0 faces the blue wall, 90 faces the far wall, -90 faces the audience.
public final class FieldPoses {

    // private constructor to prevent instantiation
    private FieldPoses() {}

    // PoseFactory instance for creating poses with degrees for heading
    private static final PoseFactory F = PoseFactory.degrees();

    // Blue scoring: tile D6, against the far wall, launcher facing the hive
    public static final Pose BLUE_SCORING_START = F.of(84, 135, -90);

    // TODO: these come from the old BlueScoring distances, measure them on the field
    public static final Pose BLUE_SCAN       = F.of(84, 115, -90);
    public static final Pose BLUE_SHOOT      = F.of(84, 134, -90);
    public static final Pose BLUE_NOT_TIPPED = F.of(84, 123, -90);
    public static final Pose BLUE_PARK_TURN  = F.of(114, 122, -90);
    public static final Pose BLUE_PARK_LANE  = F.of(114, 43, -90);
    public static final Pose BLUE_PARK       = F.of(135, 43, 0); // blue loading zone, facing the wall
}
