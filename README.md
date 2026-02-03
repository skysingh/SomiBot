# SomiBot - MyBuddy 2 Cobot Control

Training and replay system for the MyBuddy 2 collaborative robot.

## Quick Start

### Edit & Deploy to RPi

1. **Edit locally** in VSCode
   ```bash
   code /home/hp/mybuddy_dev/pymycobot/somibot/Mybuddy_train_v200_R.py
   ```

2. **Deploy to Pi**
   ```bash
   ./deploy.sh pymycobot/somibot/Mybuddy_train_v200_R.py
   ```

3. **Run on Pi**
   ```bash
   ssh er@192.168.1.80 'python3 /home/er/pymycobot/somibot/Mybuddy_train_v200_R.py'
   ```

4. **Pull changes from Pi** (if edited there)
   ```bash
   ./pull.sh pymycobot/somibot/Mybuddy_train_v200_R.py
   ```

---

### GitHub Workflow

1. **Stage changes**
   ```bash
   git add pymycobot/somibot/Mybuddy_train_v200_R.py
   ```

2. **Commit**
   ```bash
   git commit -m "Description of changes"
   ```

3. **Push to GitHub**
   ```bash
   git push
   ```

4. **Pull latest** (if working from multiple machines)
   ```bash
   git pull
   ```

---

## Connection Info

| Target | Address |
|--------|---------|
| RPi | `er@192.168.1.80` |
| GitHub Repo | `github.com/skysingh/SomiBot` |

---

## Script Features (v200_R)

- Manual teaching mode with deadman switch (GPIO 17)
- Position recording (GPIO 27)
- Left arm: Flexible gripper
- Right arm: Parallel OR Vacuum gripper (selectable)
- 5 auxiliary GPIO outputs (AUX1-AUX5)
- Save/load recordings as JSON or CSV
- Replay with configurable speed and looping

### GPIO Pin Configuration

| Pin | Function |
|-----|----------|
| 17 | Deadman switch (hold to free arms) |
| 27 | Record button |
| 22 | Left gripper toggle |
| 7 | Vacuum pump output |
| 8, 25, 24, 23, 18 | AUX1-AUX5 outputs |
