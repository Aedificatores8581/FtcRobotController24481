# Limelight code

Code that runs **on the Limelight itself** (not on the Control Hub). It lives here so it is
versioned with the robot code that depends on it.

| File | What it is | Used by |
|------|------------|---------|
| `PythonSnapScript.py` | SnapScript for pipeline slot 3: finds AprilTag clusters, returns tx/ty, draws the HUD | `opmodes/python/PythonAimBase` (Python Aim Red/Blue) |

The main TeleOps (`BlueAimingTeleOp`, `RedAimingTeleOp`) use the normal AprilTag pipeline 0, not this script.

## Deploying a change

Git does not update the Limelight. After you change the script:

1. Open the Limelight web UI (`http://limelight.local:5801`).
2. Select pipeline slot 3, pipeline type **Python SnapScript**.
3. Paste in the script and save.
4. Run Python Aim Red/Blue and check the stream HUD.

Keep the script and `PythonAimBase` in step: the output array layout is documented at the top of
the script, and the Java reads it by index.

## Tip

In the Limelight UI you can download a pipeline (`.vpr`) and the settings. Commit those in this
folder too if you want a full backup of the camera setup.
