# Lava VTOL

ComputerCraft autopilot for a Sable ship that takes off vertically on downward
Create Propulsion thrusters and cruises on a rear thruster, flown through a
Rocket Pilot **Flight Controller (peripheral)**.

- **Lift thrusters** (nozzles down, e.g. a pair at each corner) hold the height and tilt the plane to move sideways or brake.
- **Rear thrusters** (nozzles out the back) push whenever the way to go is ahead of the nose.
- The nose can't be turned (vertical and rear thrust make no yaw), so targets off to the side are reached by sliding.
- Pushes cobble from Mechanical Generators into every basin and drains the basins into the lava tank, then feeds diesel to the Liquid Burner(s) picked with B (the one under the basin), lava to the other burners and to every thruster, fullest source first. Lands by itself when the lava runs low.

## Install

```
wget https://raw.githubusercontent.com/levisnakes/cc-lava-vtol/main/vtol.lua vtol.lua
```

Wiring: a wired modem on every thruster, on the Flight Controller and on this computer, all on the same cables, every modem right-clicked red. For refuelling, each thruster's modem must be on its intake end, and the onboard tank needs a modem too.

Run `vtol` (or `vtol log` to write `vtol.log`). Tap the buttons on screen (Advanced Computer) or use the keys.

![Flight screen](docs/screen.png)

Anything stopping a good flight shows in red under the panels, in plain words. Tap **BURNERS** to choose which Liquid Burner gets diesel:

![Burner picker](docs/burners.png)

| Key | |
|---|---|
| T | take off to `TAKEOFF_HEIGHT` |
| G | go to `x z` or `x y z` (`~` = relative) |
| H | hover here |
| L | land |
| M | manual: W/S forward speed, A/D sideways, R/F up/down 5, STOP button stops |
| B | pick which Liquid Burner(s) get diesel (saved) |
| SPACE (hold) | fly forward at `SPACE_SPEED`; let go to stop. Works from a Linked Typewriter on the network (bind SPACE to a frequency) or the keyboard |
| O | thrusters off |
| Q | quit (thrusters off) |

If the program stops for any reason the Flight Controller cuts every thruster after 1 s, so don't quit in the air.
