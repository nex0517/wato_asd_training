# WATonomous ASD Assignment: Robot Navigation

My solution to the WATonomous ASD admissions assignment. A differential-drive robot (two driven wheels and a caster) with a 360° lidar has to drive to whatever point you click in Foxglove without hitting anything. It all runs in a Gazebo simulation inside Docker.

I started this knowing Python well and not much C++, ROS2, Docker or robotics, so a lot of what's written here is stuff I found out by getting it wrong first.

## How it works

Four ROS2 nodes, each feeding the next:

```
/lidar -> costmap -> /costmap -> map_memory -> /map -> planner -> /path -> control -> /cmd_vel -> robot
```

- **costmap** turns each lidar scan into a 30 × 30 m grid around the robot, with 0.1 m cells. A cell with a lidar hit gets 100, and cells within 2 m of a hit get a cost that fades from 100 down to 0 with distance.
- **map_memory** keeps one 60 × 60 m map of the whole world and pastes costmaps into it. Since the costmap turns with the robot, it needs both where the robot was and which way it was facing when that scan came in. It merges a new costmap every 1.5 m of driving.
- **planner** runs A* on that map from the robot to the goal. Stepping into a cell costs the distance plus extra for how close the cell is to an obstacle, and cells at 35 or above can't be entered at all. It replans every half second until the robot is within 0.5 m of the goal.
- **control** follows the path with pure pursuit. It picks a point 1 m ahead on the path (the "carrot") and drives along the arc that goes through it. If the carrot is more than 60° off to the side, it turns on the spot first.

One detail of this sim ended up mattering more than anything else: the robot's position (`/odom/filtered`) is the position of the lidar, not the middle of the robot. The lidar sits 1.3 m in front of the wheel axle, and the axle is what the robot actually turns around. Both of the problems below come back to that 1.3 m.

## Giving the robot more room

With my first settings, the robot kept catching the corner of a box with its back end while turning, even though the planned path went around the box fine.

At that point the costmap spread cost 1.0 m out from obstacles, and the planner blocked anything within 0.7 m, which is half the robot's width with the wheels. I had already lowered that cutoff from the 90 the guide suggests. At 90 only the cells touching an obstacle are blocked, which leaves a 1 m wide robot about 10 cm to spare.

To check it wasn't just a close call, I used a script that works out the robot's real footprint (chassis and wheels) from its pose and measures the distance to the actual obstacles from the Gazebo world file. On that route the closest distance came out to 0.00 m, so the body really was touching.

The reason is that control steers the lidar point, but the robot pivots on its wheels 1.3 m further back. It acts like a trailer: the back cuts inside the corner the front just took. On the tightest possible turn (0.5 m/s with the turn rate capped at 1 rad/s), the wheels move on a circle of radius 0.5 m, while the lidar point moves on one of radius √(0.5² + 1.3²) ≈ 1.39 m. That puts the wheels about 0.89 m inside the path. Add the 0.7 m half-width and the path needs to stay roughly 1.6 m away from anything.

The planner can't keep away from something it doesn't know about, and the costmap only marked cost out to 1.0 m, so the costmap had to change:

- costmap inflation: 1.0 m → 2.0 m
- planner cutoff: 30 → 35. With the wider inflation, a value of 35 is 1.3 m from an obstacle (100 · (1 − 1.3/2.0) = 35), so anything closer than 1.3 m is blocked.

Cells between 1.3 m and 2.0 m still cost extra, so when there's space A* stays out even further than 1.6 m.

I thought about steering from the axle instead, but the odom, the start of every path, and the "goal reached" check are all based on the lidar point. Switching would have meant changing three nodes, while fixing the clearance only took two numbers.

On the same route, the body now stays at least 0.76 m from obstacles and the path at least 1.70 m. I also checked that the narrowest gap in the world (about 3.5 m, between two of the boxes) is still passable. The downside is that goals within 1.3 m of a known obstacle get rejected now, and routes are wider than they strictly need to be.

## Getting unstuck

The first version of control had two checks for being stuck, and both just stopped the robot and gave up on the goal:

- it was driving, but the lidar point moved less than 10 cm in 5 seconds of sim time (pushing against something)
- it spun a full 360° and the carrot never came in front

Stopping is safe, but it means someone has to step in every time.

The spinning case turned out to be the interesting one. Send a goal 1 m behind the lidar and the robot starts turning to face it, and just keeps turning. Here's why:

- While spinning in place, the lidar point moves around a circle of radius 1.3 m centred on the axle.
- If the goal is less than 1.3 m from the axle, it's inside that circle, so it stays behind the lidar point whichever way the robot faces. It can never come in front.
- The goal only counts as reached when the lidar point gets within 0.5 m of it. For a goal less than 0.8 m from the axle, that never happens during a spin either.
- So a goal that close to the wheels can't be faced or reached just by turning. The 360° check caught this, but all it could do was give up.

So I added a new state, REVERSING. When a stuck check fires, the robot backs up 1 m in a straight line at 0.3 m/s instead of stopping, then tries again (turning on the spot or driving, whichever makes sense for where the carrot is). Backing up moves the wheels away from a goal that was too close to them. In the case above the goal starts 0.3 m in front of the axle. After backing up 1 m it's 1.3 m in front, which is exactly where the lidar point is, so the robot arrives.

Some limits so it can't make things worse:

- At most 2 back-ups per goal. After that it stops and ignores that goal until you click a different one, so it can't loop forever.
- If backing up doesn't make progress either, it stops right away instead of pushing.
- I shortened the "no progress" window from 5 to 3 sim seconds, so it reacts before pushing against something for too long.
- I also added a third check, for trying to spin but turning less than 10° in 3 sim seconds. It never fired in testing, because in this sim the robot slides free instead of jamming.

I tested it by driving straight into the big cylinder: back up (1 of 2), retry, back up (2 of 2), then "stuck after 2 back-ups". Sending the goal 1 m behind the lidar gave a 361° spin, one back-up, then "goal reached".

With the wider clearance, backing up is more of a safety net than the main defence, but it still comes up. A spin swings the front corners about 1.58 m out from the wheels, more than the 1.3 m cutoff. The map is also a bit behind reality: it only updates every 1.5 m, and the far sides of obstacles count as free until the lidar sees them.

## Other things I learned

- Gazebo's drive plugin keeps running the last `/cmd_vel` it got, forever. Stopping means sending an explicit zero, and control sends three in a row to be safe.
- Foxglove's teleop publishes on the same `/cmd_vel`, so control sends nothing at all while it's idle. Otherwise it would fight you.
- `odometry_spoof` keeps republishing the last pose even when the sim stops updating, so "an odom message arrived" doesn't mean the position is fresh. Control checks that the timestamp actually changed.
- Gazebo's first few lidar scans come in before the world has loaded, so they're empty. My first map merge used one of them, and `/map` stayed blank until the robot drove 1.5 m. An empty costmap doesn't count as the first merge anymore.
- Use `floor` when converting metres to cells, not a cast to int. A cast rounds toward zero, so −0.3 becomes cell 0 instead of −1.
- The guide's A* hash, `x ^ (y << 1)`, maps 360,000 cells onto about 2,000 values. Packing x and y into one 64-bit number gives every cell its own value.
- The sim runs about 8× slower than real time, so a 10 m drive takes around 3 real minutes.
- Building while Gazebo was running used up all of WSL's 3.7 GB of RAM and took Docker down with it. Now I always run `./watod down` before building.

## Known limitations

- The trailer effect is smaller now, but the tightest turns still aren't guaranteed to clear.
- Unknown space is treated as free, and once something is on the map it stays there. Nothing gets cleared.
- Goals within 1.3 m of a known obstacle are rejected.
- The goal tolerance is 0.5 m instead of the guide's 0.1 m, to match the planner, which declares the goal reached at 0.5 m.
- Backing up is blind, because control doesn't have the map. It's usually fine since it's reversing along where it just came from, but that isn't guaranteed.

## Running it

You need Linux (I used WSL2 on Windows) and Docker Engine, same as the original assignment setup.

1. Make `watod-config.local.sh` (copy `watod-config.sh`) and set `ACTIVE_MODULES="robot gazebo vis_tools"`.
2. Build and start:
   ```bash
   ./watod build
   ./watod up
   ```
3. Open Foxglove and connect to `ws://localhost:<FOXGLOVE_BRIDGE_PORT>`. The port is in `modules/.env` after the first run.
4. Import the layout from `config/wato_asd_training_foxglove_config .json` (the space in the filename is really there).
5. Use the 3D panel's publish-point tool to click a goal. It's published on `/goal_point`.
