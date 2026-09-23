# Git Workflow - Team Cobalt Chaos

- How we share code without overwriting each other. 
- Read Part 1 once, then live in Part 3.

Run these in a terminal. Android Studio has one built in: **View > Tool Windows >
Terminal** (`Alt+F12`). It opens already inside the project folder.

---

## Part 1: Set up your computer (once)
Skip this if you already have Git and the GitHub CLI installed.

### Install the tools

**macOS:**

```bash
brew install git gh
```

**Windows:** install Git from https://git-scm.com/download/win and the GitHub CLI from
https://cli.github.com, then restart Android Studio so it picks them up.

### Tell git who you are

Git stamps your name and email into every commit. Set them before your first commit —
changing them afterward means rewriting history.

```bash
git config --global user.name "Your Name"
git config --global user.email "the-email-on-your-github-account@example.com"
```

Use the email that's on your GitHub account, or GitHub won't connect the commits to you.

### Log in

```bash
gh auth login
```

Choose **GitHub.com > HTTPS > Yes > Login with a web browser**. It gives you a code to
paste into the browser.

```bash
gh auth setup-git
```

That makes `git push` reuse the login, so you never type a password.

### Windows only

```bash
git config --global core.autocrlf true
```

Windows ends lines with `\r\n` and macOS with `\n`. Without this, every file one of you
touches looks 100% changed to the other, and you'd get conflicts on lines nobody edited.

---

## Part 2: Get the code (once per computer)
Skip this if you already have the code on your computer.

```bash
git clone https://github.com/Aedificatores8581/FtcRobotController24481.git
cd FtcRobotController24481
git remote -v
```

That last command must print `Aedificatores8581`. If it prints `FIRST-Tech-Challenge`
you cloned FIRST's original repo instead of our copy. You can't push to that one. Fix it
without re-cloning:

```bash
git remote set-url origin https://github.com/Aedificatores8581/FtcRobotController24481.git
```

Then **File > Open** in Android Studio, pick the project folder, and let Gradle sync.

### How the repos relate

```
FIRST-Tech-Challenge/FtcRobotController      the official SDK, read-only to us
        |  forked
Aedificatores8581/FtcRobotController24481    our team's repo  ("origin")
        |  cloned
   your computer                             where you work
```

A fork is just a copy of a repo that lives under our organization on GitHub. It remembers
where it came from, which is how we pull in FIRST's yearly SDK updates.

---

## Part 3: The daily loop

Every feature follows the same cycle.

```bash
# 1. get everyone's latest work
git checkout master
git pull

# 2. branch, named after the feature
git checkout -b arm-pid
```

Here you are on a clean branch, ready to start your feature. Write your code in Android Studio, make changes, etc.

When finished with your feature, follow these steps:

```bash
# 3. check what changed
git status

# 4. add your changes by pointing to what files, then commit them with a reflective message
git add TeamCode/*
git commit -m "Add PID control to the arm"

# 5. push (first time on this branch)
git push -u origin arm-pid
```

Keep working. From now on it's just three commands:

```bash
git add .
git commit -m "Tune arm kP"
git push
```

When the feature is done or you're done modifying, open a pull request (Part 4). After it's merged:

```bash
git checkout master
git pull
git branch -d arm-pid
```

The `-u` in step 5 links your branch to GitHub, which is why later pushes don't need
arguments.

### Branch names

Name branches after the feature, not yourself:

good -  `arm-pid`, `auto-red-left`, `intake-subsystem`, `bug-fix-driver`, `odometry-add-strafing`
bad - `test`, `ethan2`, `my-branch`, `hi`, `new`

Be intentional with your branch naming. It should reflect what you are working on.

### If your branch lives more than a day or two

Pull master into it so you don't drift far from everyone else:

```bash
git checkout master
git pull
git checkout arm-pid
git merge master
git push
```

Merging often means small conflicts. Merging once at the end means a big one.

**You should always try to sync your branch with `master` frequently, like at the start of each work session.**

---

## Part 4: Pull requests

A pull request asks for your branch to be merged into `master`. The other programmer
reviews it first before it gets merged into `master`. This allows for code review and catching potential issues early.

You can also create a pull request in the UI after pushing your branch to GitHub with changes. It should display a prompt to create a pull request for your branch. **Ensure you point the base to `master` in our repository, not the upstream repository.**

```bash
gh pr create --base master --fill
```

It prints a link. Send it to your teammate.

>  Use that command, not the GitHub website. Because our repo is a fork, the website
> defaults the destination to **FIRST-Tech-Challenge**, you'd be publicly proposing our
> robot code to the people who write the FTC SDK. `gh pr create` always targets our repo.

Once it's approved:

```bash
gh pr merge --squash --delete-branch
```

Then tell your teammate so they can `git checkout master` and `git pull`.

---

## Part 5: When something goes wrong

**Worked on master by accident (not pushed yet):**

```bash
git branch arm-pid      # copies your commits onto a new branch
git checkout arm-pid
```

**Undo the last commit, keep the code. This is useful if you committed too early:**

```bash
git reset --soft HEAD~1
```

**Throw away changes to one file. If you want to discard your edits to a specific file:**

```bash
git checkout -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/YourFile.java
```

**Need to switch branches mid-work:**

```bash
git stash
git checkout other-branch
# later
git checkout arm-pid
git stash pop
```

**Merge conflict:** Android Studio shows both versions side by side. Pick the correct
lines, save, then `git add .` and `git commit`. If you can't tell which side is right,
ask instead of guessing — the other side is someone else's work.

Anything beyond this, ask a mentor. Don't paste in commands from the internet; several
common ones delete work permanently.

---

## Part 6: Rules

1. **Never commit directly to `master`.** Branch first, always.
2. **Our code goes in `TeamCode/`.** Never edit anything inside `FtcRobotController/` -
   those are FIRST's sample opmodes. Copy a sample into `TeamCode` if you want to use it.
   This is what keeps SDK updates from turning into a mess of conflicts.
3. **Split work by subsystem.** Two people editing one opmode at the same time produces
   conflicts that are miserable to resolve at a competition.
4. **`git pull` before you start.** Every session.
5. **Let Gradle finish syncing before you commit.**

---

## Part 7: ADB (Android Debug Bridge) Usage

If you want to use ADB to interact with your robot, here are some common commands. Ensure ADB is installed and available in your system's PATH.

```bash
# List connected devices
adb devices

# Connect to a device over Wi-Fi
adb connect DEVICE_IP

# Disconnect from a device over Wi-Fi
adb disconnect DEVICE_IP

# Install an APK on the device
adb install path/to/your.apk

# Uninstall an app from the device
adb uninstall com.your.package.name

# Start a shell on the device
adb shell

# Pull a file from the device
adb pull /sdcard/path/to/file local/path

# Push a file to the device
adb push local/path /sdcard/path/to/file
```

## Part 7: Cheat card

| Goal | Command |
|---|---|
| Start a feature | `git checkout master` → `git pull` → `git checkout -b NAME` |
| See what changed | `git status` |
| Commit | `git add .` → `git commit -m "message"` |
| Push (first time) | `git push -u origin NAME` |
| Push (after that) | `git push` |
| Open a PR | `gh pr create --base master --fill` |
| Get teammate's merged work | `git checkout master` → `git pull` |
| Update branch from master | `git merge master` (while on your branch) |
| Which branch am I on? | `git branch` |
| Recent history | `git log --oneline -10` |
| Delete a merged branch | `git branch -d NAME` |
| Disconnect from a device over Wi-Fi | `adb disconnect DEVICE_IP` |
| Connect to a device over Wi-Fi | `adb connect DEVICE_IP` |


---

## For **mentors**: Updating the FTC SDK

FIRST ships a new SDK most seasons (v11.2 in July, v12.0 in September). One person does
this, then the students pull.

```bash
git remote add upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git
git remote set-url --push upstream DISABLED      # blocks accidental pushes to FIRST

git fetch upstream
git log origin/master..upstream/master --oneline # empty output = we're current

git checkout master
git pull
git merge upstream/master
# resolve conflicts, then BUILD AND TEST ON THE ROBOT
git push origin master
```

GitHub's "Sync fork" button does the same thing, but only when our master has no commits
of its own. Ours does, so use the CLI.
