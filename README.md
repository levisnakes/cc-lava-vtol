# Lava VTOL

ComputerCraft autopilot for a Sable ship that takes off vertically on downward
Create Propulsion thrusters and cruises on a rear thruster, flown through a
Rocket Pilot **Flight Controller (peripheral)**.

- **Lift thrusters** (nozzles down, e.g. a pair at each corner) hold the height and tilt the plane to move sideways or brake.
- **Rear thrusters** (nozzles out the back) push whenever the way to go is ahead of the nose.
- The nose can't be turned (vertical and rear thrust make no yaw), so targets off to the side are reached by sliding.
- Optional: keeps every thruster topped up from an onboard lava tank and lands by itself when the tank runs low.

## Install

```
wget https://raw.githubusercontent.com/levisnakes/cc-lava-vtol/main/vtol.lua vtol.lua
```

Wiring: a wired modem on every thruster, on the Flight Controller and on this computer, all on the same cables, every modem right-clicked red. For refuelling, each thruster's modem must be on its intake end, and the onboard tank needs a modem too.

Run `vtol` (or `vtol log` to write `vtol.log`).

| Key | |
|---|---|
| T | take off to `TAKEOFF_HEIGHT` |
| G | go to `x z` or `x y z` (`~` = relative) |
| H | hover here |
| L | land |
| M | manual: W/S forward speed, A/D sideways, R/F up/down 5, Space stop |
| O | thrusters off |
| Q | quit (thrusters off) |

If the program stops for any reason the Flight Controller cuts every thruster after 1 s, so don't quit in the air.
