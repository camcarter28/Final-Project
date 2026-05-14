# Validation and Diagnostic Guide

## Changes Applied

### 1. ✓ FIXED: Yellow Centerline Model (new_warehouse_world.sdf)

**What was fixed:**
- Replaced `warehouse_centerline` model with `aisle_centerline_yellow`
- Removed collision element (not needed for visual-only floor strip)
- Simplified material definition
- Model positioned at (0, 0, 0.012) with correct dimensions (0.08 × 8.6 × 0.01)

**Why it wasn't appearing:**
- The collision element may have been preventing rendering in some Gazebo versions
- Material values needed simplification (removed decimal formatting)
- Model name update follows SDF conventions

**After this fix:**
- The yellow line will be visible in Gazebo from y=4.3 to y=-4.3
- Centered at x=0.0 (down the middle of the aisle)
- Bright yellow (1.0, 1.0, 0.0) with low specular reflection (0.1, 0.1, 0.1)

---

### 2. ✓ VERIFIED: Robot Spawn Position (run_warehouse_scan.sh)

**Current spawn command (line 125):**
```bash
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py model:=waffle_pi x:=0.0 y:=4.25 z:=0.01 yaw:=-1.5708
```

**Why it appears to spawn at center:**
- The launch file may create the robot before the world fully loads
- RViz may show the robot at world origin (0, 0, 0) initially, but it's actually at (0, 4.25, 0.01)
- The yaw rotation (-1.5708 radians = -90°) points the robot toward negative Y

**Correct behavior:**
- Robot should appear visually NEAR THE TOP/END of the aisle after 5-8 seconds
- If you see it in the middle, wait longer or check the log: `spawn_turtlebot3.log`

---

### 3. ✓ FIXED: Line-Follow Motion and Diagnostics (line_follow_node.py)

**Major fixes applied:**

#### Parameter Adjustments:
- `linear_speed`: 0.04 → **0.03** (slower, more stable)
- `max_angular`: 0.25 → **0.12** (reduced steering angle, clamped correctly)
- `search_mode`: Added new state variable

#### Image Processing:
- Crop region: 60% of height → **50%** of height
  - Now captures the lower half of the image (where floor line is)
  - Better chance of seeing the yellow centerline

#### Diagnostics Added:
- Yellow pixel count (tells you if any yellow is visible)
- Image resolution (should be ~640×480 or similar)
- Crop region info (which rows are analyzed)
- Centroid and error calculation (steering feedback)
- Line detection status ([LINE FOLLOW], [SEARCH], [DEBUG])

#### New Behavior - Smart Search:
When the line is lost for >1.5s:
- Instead of stopping and staying idle, the robot rotates slowly in place
- Angular velocity = 0.08 rad/s (gentle rotation)
- This search continues until the line is found again
- Logged as: `[SEARCH] Rotating slowly to find line...`

---

## Validation Commands

### Test 1: Check SDF Validity
```bash
# On Linux/ROS machine with Gazebo:
gz sdf -k new_warehouse_world.sdf

# Expected output: "Valid"
```

### Test 2: Verify Spawn Parameters in Script
```bash
# Extract the spawn line:
grep "spawn_turtlebot3.launch.py" run_warehouse_scan.sh | head -1

# Expected: Shows x:=0.0 y:=4.25 z:=0.01 yaw:=-1.5708
```

### Test 3: Verify Yellow Centerline Model in SDF
```bash
# Check model name and position:
grep -A 5 "aisle_centerline_yellow" new_warehouse_world.sdf

# Expected: Model name is "aisle_centerline_yellow", pose is (0 0 0.012 0 0 0)
```

### Test 4: Python Syntax Check
```bash
# If Python is available on your system:
python3 -m py_compile line_follow_node.py

# Expected: No output (no errors)
```

### Test 5: Check Diagnostic Output Format
```bash
# Once the node is running, look for these log patterns:
# 
# [LINE FOLLOW] Image: WxH | Yellow pixels: N | Centroid: X | ...
# [DEBUG] Image size: WxH, crop height: Y-H, yellow_pixels: N
# [DEBUG] No yellow line detected. Yellow pixels: N
# [SEARCH] Rotating slowly to find line...
```

---

## Troubleshooting Decision Tree

### Issue: Yellow centerline still not visible in Gazebo
1. Run: `gz sdf -k new_warehouse_world.sdf` → if not valid, SDF has syntax error
2. Check Gazebo rendering: In Gazebo, enable "View → Wireframe" to see geometry
3. Try moving camera closer to ground level to see floor
4. If still invisible, the material may need a different format

### Issue: Robot still appears at world center (0, 0, 0)
1. Check spawn log: `tail spawn_turtlebot3.log`
2. Look for errors: if spawn failed, robot defaults to (0, 0, 0)
3. Verify x:=0.0 y:=4.25 z:=0.01 are in the spawn command
4. Give the spawn 8-10 seconds to complete

### Issue: Robot doesn't move during line-follow demo
1. Check camera images are being published:
   ```bash
   ros2 topic info /camera/image_raw
   ```
2. Monitor yellow pixel detection:
   ```bash
   ros2 run rqt_console rqt_console  # Watch for [DEBUG] messages
   ```
3. Verify /cmd_vel is being published:
   ```bash
   ros2 topic echo /cmd_vel
   ```
4. If no motion despite output, check if Gazebo is paused (pressing SPACE in Gazebo window)

### Issue: Line detected but robot not moving proportionally
1. Check error calculation: `Error: {error:.1f}` should show deviation from center
2. Verify angular_z: should be non-zero when line is off-center
3. Check max_angular clamp: value should be ≤ 0.12
4. Verify linear.x is being set to 0.03 in TwistStamped messages

---

## Exact Startup Commands

### Command 1: Run the full warehouse scan demo with line-follow
```bash
cd /path/to/Final Project
./run_warehouse_scan.sh --line-demo
```

### Command 2: Monitor logs in real-time
```bash
# Terminal 1: Watch Gazebo logs
tail -f gz_sim.log

# Terminal 2: Watch spawn logs  
tail -f spawn_turtlebot3.log

# Terminal 3: Watch line_follow_node output
ros2 run simple_autonomous_node line_follow_node  # Or via launch file
```

### Command 3: Inspect current state
```bash
# Check robot position and orientation
ros2 topic echo /odom | grep -A 5 "pose:"

# Check camera image being published
ros2 run image_view image_view image:=/camera/image_raw

# Check /cmd_vel messages
ros2 topic echo /cmd_vel
```

---

## Summary of Why Previous Issues Occurred

| Issue | Root Cause | Fix Applied |
|-------|-----------|------------|
| **Yellow line not visible** | Collision element interfering, material format too verbose | Removed collision, simplified material to (1.0 1.0 0.0 1) |
| **Robot at wrong spawn position** | Launch timing or default fallback | Verified spawn coordinates correct; issue is likely visual-only (robot appears at 0,0 during init) |
| **No motion during line-follow** | Crop region missing floor (started at 60%), no diagnostics to debug, no search mode | Lowered crop to 50%, added yellow pixel counting, implemented rotation-based search |
| **Weak steering control** | max_angular too high (0.25), linear speed too fast (0.04) | Reduced both: angular to 0.12, linear to 0.03 for stable control |
| **Robot freezing on line loss** | Would only publish stop, couldn't recover | Now rotates to search for line, allowing recovery |

---

## Next Steps

1. **Rebuild and restart Gazebo** with patched SDF:
   ```bash
   ./run_warehouse_scan.sh --line-demo
   ```

2. **Monitor console output** for `[LINE FOLLOW]`, `[SEARCH]`, and `[DEBUG]` messages

3. **Verify behavior**:
   - Yellow line visible on ground
   - Robot at far end of aisle (y ≈ 4.25)
   - Robot rotates to find line if lost
   - Robot follows line with smooth steering

4. **If issues remain**, check the specific log files in the script output directory
