-- Lava VTOL: flies a Sable ship that takes off on downward thrusters and
-- cruises on a rear thruster, through a Rocket Pilot Flight Controller
-- (peripheral) on the same wired network as this computer.
--
--   Lift thrusters (nozzles down) hold the height and tilt the plane to
--   move sideways or brake. Forward thrusters (nozzles out the back) push
--   whenever the way to go is ahead of the nose. Nothing here can turn the
--   nose (vertical and rear thrust make no yaw), so the plane slides toward
--   targets that are not dead ahead.
--
-- Keys:  T take off   G go to (x z, or x y z, ~ = relative)   H hover here
--        L land       M manual   O off   Q quit
-- Manual: W/S forward speed   A/D sideways speed   R/F up/down 5 m
--         Space stop moving
--
-- Optional fuel: give the onboard lava tank and each thruster's intake end
-- a wired modem on this network and it keeps every thruster topped up, and
-- lands by itself when the tank runs low.

TAKEOFF_HEIGHT = 15       -- blocks above the ground after T
CRUISE_SPEED = 25         -- top speed for G (blocks/s)
MANUAL_STEP = 2           -- W/S/A/D change the manual speed by this much
MAX_TILT = 12             -- degrees the plane may lean to move or brake
SAFETY_TILT = 40          -- above this it drops everything and levels out
CLIMB_SPEED = 6
DESCENT_SPEED = 4
LAND_SPEED_LOW = 0.6      -- touchdown speed
REAR_MAX = 1.0            -- most the rear thrusters may be throttled to

FUEL_TANK = nil           -- the lava tank (basins drain into it, thrusters fuel from it); nil = any tank with lava
FUEL = "minecraft:lava"
DIESEL = "createdieselgenerators:diesel"
DIESEL_BURNERS = nil      -- burners that get diesel, e.g. { "createpropulsion:liquid_burner_block_entity_3" }; nil = the ones picked with B
OTHER_BURNER_FUEL = "minecraft:lava" -- what every other Liquid Burner gets; nil = nothing
BURNER_PUSH_MB = 250      -- pushed into each Liquid Burner per round (burners come first)
BURNER_RESERVE_MB = 1000  -- while a lava burner still wants lava, thrusters leave this much
LOW_FUEL_MB = 2000        -- land when the onboard tank drops below this
LOW_THRUSTER_MB = 200     -- with no tank: land when a lift thruster drops below this

LOG = false               -- write vtol.log (every 5 ticks) for debugging
AUTO_UPDATE = true        -- install new versions from GitHub automatically, keeping these settings

-- Settings in /config.lua override the ones above. Use it for anything you
-- don't want to retype: it survives reinstalls and updates, and one file can
-- be shared by every computer (each program ignores settings it doesn't use).
do
  local path = "/config.lua"
  if fs.exists(path) then
    local fn, err = loadfile(path, "t", _ENV)
    if fn then
      local ok, runErr = pcall(fn)
      if not ok then printError("config.lua: " .. tostring(runErr)) sleep(3) end
    else
      printError("config.lua: " .. tostring(err))
      sleep(3)
    end
  end
end

-- ============================================================ setup

local VERSION = "1.6.0"
local args = { ... }
if args[1] == "log" then LOG = true end

local fc = peripheral.find("flight_computer")
if not fc then
  error("No Flight Controller found. Put a wired modem on it (right-click it red) on the same cables as this computer.", 0)
end

-- control gains
local K_ALT, K_VZ, K_IZ = 1.0, 2.5, 0.6
local K_POS, K_VH = 0.5, 0.8
local ATT_W, ATT_Z = 2.5, 0.9      -- attitude response (rad/s) and damping
local MIN_T = 0.01                 -- lift floor in flight: thrusters spool up slowly from 0

local F = {                        -- flight state, shared by the loops
  mode = "off", msg = "ready", target = nil,
  integ = 0, touch = 0,
  vF = 0, vS = 0,                  -- manual speeds (forward, sideways)
  st = nil, vel = nil, prevPos = nil, lastTime = nil,
  rear = 0, liftN = 0, rearN = 0, maxTiltSeen = 0,
  tank = nil, tankMb = nil, sourceN = 0, lowFuel = false,
  running = true,
}
local logf = nil

-- ============================================================ math

local function v3(x, y, z) return { x = x or 0, y = y or 0, z = z or 0 } end
local function add(a, b) return v3(a.x + b.x, a.y + b.y, a.z + b.z) end
local function sub(a, b) return v3(a.x - b.x, a.y - b.y, a.z - b.z) end
local function mul(a, s) return v3(a.x * s, a.y * s, a.z * s) end
local function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
local function cross(a, b) return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x) end
local function len(a) return math.sqrt(dot(a, a)) end
local function norm(a) local l = len(a) if l < 1e-9 then return v3(0, 1, 0) end return mul(a, 1 / l) end
local function clamp(v, lo, hi) if v < lo then return lo elseif v > hi then return hi end return v end

-- q maps ship-local to world; inverse = conjugate
local function rotate(q, v, inverse)
  local qx, qy, qz, qw = q.x, q.y, q.z, q.w
  if inverse then qx, qy, qz = -qx, -qy, -qz end
  local tx = 2 * (qy * v.z - qz * v.y)
  local ty = 2 * (qz * v.x - qx * v.z)
  local tz = 2 * (qx * v.y - qy * v.x)
  return v3(v.x + qw * tx + (qy * tz - qz * ty),
            v.y + qw * ty + (qz * tx - qx * tz),
            v.z + qw * tz + (qx * ty - qy * tx))
end

-- Solves an n x n system (n <= 3) by Gaussian elimination; nil if singular.
local function solve(m, b)
  local n = #b
  local a = {}
  for r = 1, n do
    a[r] = {}
    for c = 1, n do a[r][c] = m[r][c] end
    a[r][n + 1] = b[r]
  end
  for c = 1, n do
    local p = c
    for r = c + 1, n do if math.abs(a[r][c]) > math.abs(a[p][c]) then p = r end end
    if math.abs(a[p][c]) < 1e-12 then return nil end
    a[c], a[p] = a[p], a[c]
    for r = c + 1, n do
      local f = a[r][c] / a[c][c]
      for k = c, n + 1 do a[r][k] = a[r][k] - f * a[c][k] end
    end
  end
  local x = {}
  for r = n, 1, -1 do
    local s = a[r][n + 1]
    for k = r + 1, n do s = s - a[r][k] * x[k] end
    x[r] = s / a[r][r]
  end
  return x
end

-- ============================================================ thrust mixing

-- Throttles for the lift thrusters giving total lift `lift` along ship up and
-- torque (tx, tz) about the ship's x and z axes. Active set: solve for the
-- smallest change from an even throttle, pin anything that leaves [lo, 1],
-- solve again for the rest. When all three can't be met, torque wins: the
-- lift row is dropped so the plane stays level and the height loop recovers.
local function mixLift(lifts, lift, tx, tz, lo)
  local n = #lifts
  local rows = {}
  local total = 0
  for j, t in ipairs(lifts) do
    local rd = cross(t.offset, t.direction)
    rows[j] = { t.maxForce * t.direction.y, t.maxForce * rd.x, t.maxForce * rd.z }
    total = total + t.maxForce * math.max(t.direction.y, 0)
  end
  local base = clamp(lift / math.max(total, 1e-6), lo, 1)
  local out, fixed = {}, {}
  for j = 1, n do out[j] = base end
  local goals = { lift, tx, tz }

  for useLift = 1, 0, -1 do
    local first = useLift == 1 and 1 or 2
    for _ = 1, n + 1 do
      -- residual each constraint still needs from the free thrusters
      local need = {}
      for k = first, 3 do
        local s = goals[k]
        for j = 1, n do s = s - rows[j][k] * out[j] end
        need[#need + 1] = s
      end
      -- minimum-norm correction over the free thrusters: d = A' (A A')^-1 need
      local m = {}
      for a = first, 3 do
        local row = {}
        for b = first, 3 do
          local s = 0
          for j = 1, n do if not fixed[j] then s = s + rows[j][a] * rows[j][b] end end
          row[#row + 1] = s
        end
        m[#m + 1] = row
      end
      local y = solve(m, need)
      if not y then break end
      local moved = false
      for j = 1, n do
        if not fixed[j] then
          local d = 0
          for k = first, 3 do d = d + rows[j][k] * y[k - first + 1] end
          local t = out[j] + d
          if t < lo then out[j], fixed[j], moved = lo, true, true
          elseif t > 1 then out[j], fixed[j], moved = 1, true, true
          else out[j] = t end
        end
      end
      if not moved then return out end
    end
    if useLift == 1 then
      -- couldn't meet lift and torque together: retry, torque only
      for j = 1, n do out[j], fixed[j] = base, nil end
    end
  end
  return out
end

-- ============================================================ modes

local function setMode(m, why)
  F.mode, F.msg, F.touch = m, why or m, 0
  if m == "off" then
    F.target = nil
    pcall(fc.setAllThrust, 0)
  end
end

local function here()
  local p = F.st and F.st.position
  return p and v3(p.x, p.y, p.z) or nil
end

local function takeOff()
  local p = here()
  if not p or not F.st.onShip then F.msg = "not on a ship" return end
  local hover = F.st.hoverThrottle
  if hover and hover > 0.95 then
    F.msg = string.format("too heavy: hovering needs %d%% of the lift thrusters", math.floor(hover * 100 + 0.5))
    return
  end
  local h = F.st.heightAboveGround or 0
  if F.mode == "off" then F.integ = 0 end
  F.target = v3(p.x, p.y - h + TAKEOFF_HEIGHT, p.z)
  setMode("takeoff", string.format("taking off to %.0f", F.target.y))
end

local function hoverHere()
  local p = here()
  if not p then return end
  if F.mode == "off" then F.integ = 0 end
  F.target = p
  setMode("hover", "hovering")
end

local function goTo(x, y, z)
  if F.mode == "off" then F.msg = "take off first (T)" return end
  local p = here()
  F.target = v3(x, y or (F.target and F.target.y) or p.y, z)
  setMode("goto", string.format("going to %.0f %.0f %.0f", F.target.x, F.target.y, F.target.z))
end

local function land(why)
  local p = here()
  if not p or F.mode == "off" then return end
  F.target = p
  setMode("land", why or "landing")
end

local function manual()
  if F.mode == "off" then F.msg = "take off first (T)" return end
  local p = here()
  F.target = v3(p.x, (F.target and F.target.y) or p.y, p.z)
  F.vF, F.vS = 0, 0
  setMode("manual", "manual: W/S A/D R/F, space stops")
end

-- ============================================================ control step

local function velocityFrom(s, dt, gap)
  local p = s.position
  if F.prevPos and F.vel and not gap then
    local a = 0.6
    F.vel = v3(F.vel.x + a * ((p.x - F.prevPos.x) / dt - F.vel.x),
               F.vel.y + a * ((p.y - F.prevPos.y) / dt - F.vel.y),
               F.vel.z + a * ((p.z - F.prevPos.z) / dt - F.vel.z))
  else
    F.vel = v3(s.velocity.x, s.velocity.y, s.velocity.z)
  end
  F.prevPos = v3(p.x, p.y, p.z)
end

-- Wanted vertical acceleration for the current mode.
local function verticalAccel(s, p, v, dt)
  local vzT
  if F.mode == "land" then
    local h = s.heightAboveGround
    vzT = -(h and clamp(h * 0.4, LAND_SPEED_LOW, DESCENT_SPEED) or LAND_SPEED_LOW)
    if h and h < 0.6 and math.abs(v.y) < 0.3 then F.touch = F.touch + 1 else F.touch = 0 end
  elseif F.mode == "level" then
    vzT = 0
  else
    local dy = F.target.y - p.y
    vzT = clamp(dy * K_ALT, -DESCENT_SPEED, CLIMB_SPEED)
    -- slow down in time to stop on the height
    local brake = math.sqrt(2 * 2.0 * math.abs(dy))
    if math.abs(vzT) > brake then vzT = (vzT < 0 and -1 or 1) * brake end
  end
  local ez = vzT - v.y
  F.integ = clamp(F.integ + K_IZ * ez * dt, -4, 4)
  return K_VZ * ez + F.integ
end

-- Wanted horizontal velocity for the current mode.
local function horizontalTarget(p, fwH, rightH, g)
  if F.mode == "manual" then
    return add(mul(fwH, F.vF), mul(rightH, F.vS))
  end
  if F.mode == "level" or not F.target then return v3() end
  local d = v3(F.target.x - p.x, 0, F.target.z - p.z)
  local dist = len(d)
  if dist < 1e-3 then return v3() end
  local top = F.mode == "goto" and CRUISE_SPEED or 2
  -- braking only comes from leaning back, so plan on half of that
  local brakeA = 0.5 * g * math.tan(math.rad(MAX_TILT))
  local speed = math.min(top, math.sqrt(2 * brakeA * dist), K_POS * dist + 0.3)
  return mul(d, speed / dist)
end

local function step(s)
  F.st = s
  local dt, gap = 0.05, false
  if F.lastTime then
    gap = s.time - F.lastTime > 5
    dt = clamp((s.time - F.lastTime) * 0.05, 0.05, 0.25)
  end
  F.lastTime = s.time
  velocityFrom(s, dt, gap)

  -- sort thrusters: lift pushes along ship up, forward pushes level
  -- (also while off, so the screen shows the wiring before take-off)
  local lifts, rears = {}, {}
  local fLocal = v3()
  for _, t in ipairs(s.thrusters or {}) do
    if t.direction.y > 0.7 then lifts[#lifts + 1] = t
    elseif math.abs(t.direction.y) < 0.3 then
      rears[#rears + 1] = t
      fLocal = add(fLocal, mul(t.direction, t.maxForce))
    end
  end
  F.liftN, F.rearN = #lifts, #rears
  if F.mode == "off" or not s.onShip then return end
  if #lifts == 0 then setMode("off", "no lift thrusters found") return end

  if s.tilt > SAFETY_TILT and F.mode ~= "level" then setMode("level", string.format("tilt %.0f - levelling", s.tilt)) end
  if F.mode == "level" and s.tilt < 5 then hoverHere() end
  F.maxTiltSeen = math.max(F.maxTiltSeen, s.tilt)

  local g, m, q, up = s.gravity, s.mass, s.orientation, s.up
  local p, v = s.position, F.vel
  local rearMax = len(fLocal)
  local fw = rearMax > 0 and rotate(q, norm(fLocal)) or rotate(q, v3(0, 0, 1))
  local fwH = norm(v3(fw.x, 0, fw.z))
  local rightH = v3(-fwH.z, 0, fwH.x)

  local az = verticalAccel(s, p, v, dt)
  if F.mode == "land" and F.touch >= 20 then setMode("off", "landed") return end

  local vT = horizontalTarget(p, fwH, rightH, g)
  local ah = mul(sub(vT, v3(v.x, 0, v.z)), K_VH)

  -- forward thrust takes the part of the push that is ahead of the nose
  local rearH = rearMax * math.sqrt(fw.x * fw.x + fw.z * fw.z)
  local tr = 0
  if rearH > 0 then tr = clamp(m * dot(ah, fwH) / rearH, 0, REAR_MAX) end
  local rearForce = mul(fw, rearMax * tr)

  -- lift covers the rest, leaning at most MAX_TILT
  local L = sub(v3(m * ah.x, m * (g + az), m * ah.z), rearForce)
  L.y = math.max(L.y, 0.2 * m * g)
  local hMax = L.y * math.tan(math.rad(MAX_TILT))
  local hLen = math.sqrt(L.x * L.x + L.z * L.z)
  if hLen > hMax then L.x, L.z = L.x * hMax / hLen, L.z * hMax / hLen end
  local want = norm(L)
  local lift = L.y / math.max(up.y, 0.5)

  -- attitude: turn ship up toward `want`
  local e = rotate(q, cross(up, want), true)
  local w = s.bodyAngularVelocity
  local I = s.inertia or v3(m * 4, m * 4, m * 4)
  local kp, kd = ATT_W * ATT_W, 2 * ATT_Z * ATT_W
  local tx = I.x * (kp * e.x - kd * w.x)
  local tz = I.z * (kp * e.z - kd * w.z)
  -- the rear thrusters' own pitch/roll torque, cancelled up front
  for _, t in ipairs(rears) do
    local rt = mul(cross(t.offset, t.direction), t.maxForce * tr)
    tx, tz = tx - rt.x, tz - rt.z
  end

  local thr = mixLift(lifts, lift, tx, tz, MIN_T)
  local cmd = {}
  for _, t in ipairs(s.thrusters) do cmd[t.index] = 0 end
  for j, t in ipairs(lifts) do cmd[t.index] = thr[j] end
  for _, t in ipairs(rears) do cmd[t.index] = tr end
  F.rear = tr
  fc.setThrusts(cmd)

  if F.mode == "takeoff" and math.abs(F.target.y - p.y) < 0.5 and math.abs(v.y) < 0.3 then
    setMode("hover", "up - G to go somewhere, M to fly by hand")
  elseif F.mode == "goto" then
    local d = len(v3(F.target.x - p.x, F.target.y - p.y, F.target.z - p.z))
    if d < 1.0 and len(v) < 0.5 then
      F.target = v3(p.x, F.target.y, p.z)
      setMode("hover", "arrived")
    end
  end

  if logf and s.time % 5 == 0 then
    logf.writeLine(string.format("%d %s p=%.1f,%.1f,%.1f v=%.2f,%.2f,%.2f tilt=%.1f lift=%.0f rear=%.2f hag=%s",
      s.time, F.mode, p.x, p.y, p.z, v.x, v.y, v.z, s.tilt, lift, tr, tostring(s.heightAboveGround)))
    logf.flush()
  end
end

-- ============================================================ fuel

local function thrusterNames()
  local seen, out = {}, {}
  for _, name in ipairs(peripheral.getNames()) do
    if not seen[name] and peripheral.hasType(name, "thruster") then
      seen[name] = true
      out[#out + 1] = name
    end
  end
  return out
end

-- Liquid burners: CC names them after the block entity
-- ("createpropulsion:liquid_burner_block_entity"), so match on part of it.
local function isBurner(name)
  for _, ty in ipairs({ peripheral.getType(name) }) do
    if type(ty) == "string" and ty:find("liquid_burner", 1, true) then return true end
  end
  return false
end

local function burnerNames()
  local seen, out = {}, {}
  for _, name in ipairs(peripheral.getNames()) do
    if not seen[name] then
      seen[name] = true
      if isBurner(name) then out[#out + 1] = name end
    end
  end
  table.sort(out)
  return out
end

-- Which burners get diesel: DIESEL_BURNERS if set, else the ones picked with
-- B (saved in /vtol_burners.cfg so they survive restarts and updates).
local BURNER_FILE = "/vtol_burners.cfg"
local dieselPicked = {}
do
  if fs.exists(BURNER_FILE) then
    local f = fs.open(BURNER_FILE, "r")
    local data = textutils.unserialize(f.readAll())
    f.close()
    if type(data) == "table" then for _, n in ipairs(data) do dieselPicked[n] = true end end
  end
end

local function isDieselBurner(name)
  if DIESEL_BURNERS then
    for _, n in ipairs(DIESEL_BURNERS) do if n == name then return true end end
    return false
  end
  return dieselPicked[name] == true
end

local function burnerNumber(name) return "#" .. (name:match("_(%d+)$") or "?") end

-- B: list the burners, type the number(s) of the one(s) under the basin.
local function pickDieselBurners()
  local burners = burnerNames()
  term.setBackgroundColour(colours.black)
  term.clear()
  term.setCursorPos(1, 1)
  print("Which burner(s) get diesel?")
  print("")
  if #burners == 0 then
    print("No Liquid Burners on the network.")
    print("Press any key.")
    os.pullEvent("char")
    return
  end
  for _, b in ipairs(burners) do
    print(burnerNumber(b) .. "  " .. b .. (isDieselBurner(b) and "  (diesel now)" or ""))
  end
  print("")
  print("Type the number(s), e.g. 3 or 3 5.")
  print("The rest get " .. (OTHER_BURNER_FUEL and (OTHER_BURNER_FUEL:match(":(.+)$") or OTHER_BURNER_FUEL) or "nothing") .. ".")
  write("> ")
  local picked, list = {}, {}
  for word in (read() or ""):gmatch("%d+") do
    for _, b in ipairs(burners) do
      if b:match("_(%d+)$") == word and not picked[b] then
        picked[b] = true
        list[#list + 1] = b
      end
    end
  end
  dieselPicked = picked
  local f = fs.open(BURNER_FILE, "w")
  f.write(textutils.serialize(list))
  f.close()
  if DIESEL_BURNERS then
    print("Note: DIESEL_BURNERS is set at the top of the file and wins over this.")
    sleep(2)
  end
  F.msg = #list > 0 and ("diesel burners: " .. #list) or "no burner gets diesel"
end

-- How much of `fluid` a block holds, and whether it also holds anything else.
local function fluidIn(name, fluid)
  local ok, tanks = pcall(peripheral.call, name, "tanks")
  if not ok or type(tanks) ~= "table" then return nil, false end
  local total, other = 0, false
  for _, t in pairs(tanks) do
    if t.name == fluid then total = total + (t.amount or 0)
    elseif (t.amount or 0) > 0 then other = true end
  end
  return total, other
end

local function lavaIn(name) return (fluidIn(name, FUEL)) end

local function isBasin(name)
  for _, ty in ipairs({ peripheral.getType(name) }) do
    if type(ty) == "string" and ty:find("basin", 1, true) then return true end
  end
  return false
end

-- Basins and the tank their lava goes into. The tank is FUEL_TANK, or the
-- fluid block (not a basin, burner or thruster) already holding the most lava.
-- A tank holding something else (the diesel tank) is never picked.
local function basinsAndStore()
  local seen, basins, store, best = {}, {}, nil, -1
  for _, name in ipairs(peripheral.getNames()) do
    if not seen[name] then
      seen[name] = true
      if peripheral.hasType(name, "fluid_storage") and not peripheral.hasType(name, "thruster")
        and not isBurner(name) then
        if isBasin(name) then
          basins[#basins + 1] = name
        elseif not FUEL_TANK then
          local mb, other = fluidIn(name, FUEL)
          mb = mb or 0
          if not other and mb > best then store, best = name, mb end
        end
      end
    end
  end
  if FUEL_TANK then store = peripheral.isPresent(FUEL_TANK) and FUEL_TANK or nil end
  return basins, store
end

-- Create Cobblestone Mechanical Generators: a one-slot inventory from any side.
local function generatorNames()
  local seen, out = {}, {}
  for _, name in ipairs(peripheral.getNames()) do
    if not seen[name] then
      seen[name] = true
      for _, ty in ipairs({ peripheral.getType(name) }) do
        if type(ty) == "string" and ty:find("mechanical_generator", 1, true) then
          out[#out + 1] = name
          break
        end
      end
    end
  end
  return out
end

-- Cobble from the generators into every basin (a basin holds 64), and lava
-- out of every basin into the tank, so mixers on board never stall.
local function runBasins()
  local basins, store = basinsAndStore()
  local gens = generatorNames()
  F.basinN, F.generatorN = #basins, #gens
  if #basins == 0 then return end
  local jobs = {}
  for i, b in ipairs(basins) do
    if #gens > 0 then
      local gen = gens[(i - 1) % #gens + 1]
      jobs[#jobs + 1] = function() pcall(peripheral.call, gen, "pushItems", b, 1, 64) end
    end
    if store then
      jobs[#jobs + 1] = function() pcall(peripheral.call, store, "pullFluid", b, nil, FUEL) end
    end
  end
  if #jobs > 0 then parallel.waitForAll(table.unpack(jobs)) end
end

-- Everything on the network that can hold fluid, apart from thrusters and
-- burners (they only take fuel): tanks, vessels, basins. Returns the ones
-- holding `fluid` (lava by default), fullest first, and whether any fluid
-- block was found at all.
local function fuelSources(fluid)
  fluid = fluid or FUEL
  local names = {}
  if FUEL_TANK and fluid == FUEL then
    if peripheral.isPresent(FUEL_TANK) then names[1] = FUEL_TANK end
  else
    local seen = {}
    for _, name in ipairs(peripheral.getNames()) do
      if not seen[name] then
        seen[name] = true
        if peripheral.hasType(name, "fluid_storage") and not peripheral.hasType(name, "thruster")
          and not isBurner(name) then
          names[#names + 1] = name
        end
      end
    end
  end
  local out = {}
  for _, name in ipairs(names) do
    local mb = fluidIn(name, fluid)
    if mb and mb > 0 then out[#out + 1] = { name = name, mb = mb } end
  end
  table.sort(out, function(a, b) return a.mb > b.mb end)
  return out, #names > 0
end

local function fuelLoop()
  while F.running do
    runBasins()
    local sources, anyTank = fuelSources()
    local total = 0
    for _, s in ipairs(sources) do total = total + s.mb end
    F.tankMb = anyTank and total or nil
    F.tank = sources[1] and sources[1].name or nil
    F.sourceN = #sources

    -- 1. burners first. The diesel burners (picked with B) get DIESEL, every
    -- other burner gets OTHER_BURNER_FUEL. A burner that takes a whole push
    -- is still hungry; one that takes less is (nearly) full. A burner passes
    -- what it can't hold to the next one in a straight line, so the end
    -- burner feeds a row.
    local burners = burnerNames()
    local dSources = fuelSources(DIESEL)
    local dTotal = 0
    for _, s in ipairs(dSources) do dTotal = dTotal + s.mb end
    local hungry = false -- a lava burner still wants lava
    local jobs = {}
    for _, b in ipairs(burners) do
      local fluid = isDieselBurner(b) and DIESEL or OTHER_BURNER_FUEL
      local srcs = fluid == FUEL and sources or (fluid == DIESEL and dSources or nil)
      if fluid and srcs and #srcs > 0 then
        jobs[#jobs + 1] = function()
          local want = BURNER_PUSH_MB
          for _, s in ipairs(srcs) do
            if want <= 0 then break end
            local ok, moved = pcall(peripheral.call, s.name, "pushFluid", b, want, fluid)
            if ok and type(moved) == "number" then
              want = want - moved
              s.mb = s.mb - moved
              if fluid == FUEL then total = total - moved end
              if fluid == DIESEL then dTotal = dTotal - moved end
            end
          end
          if want <= 0 and fluid == FUEL then hungry = true end
        end
      end
    end
    if #jobs > 0 then parallel.waitForAll(table.unpack(jobs)) end
    F.burnerN, F.burnersHungry, F.burnerList = #burners, hungry, burners
    F.burnerFuelMb = dTotal

    -- 2. thrusters get the lava. While a lava burner is still hungry they
    -- leave BURNER_RESERVE_MB behind for it.
    local spare = hungry and math.max(0, total - BURNER_RESERVE_MB) or math.huge
    if #sources > 0 and spare > 0 then
      local jobs = {}
      for _, t in ipairs(thrusterNames()) do
        jobs[#jobs + 1] = function()
          local okA, amount = pcall(peripheral.call, t, "getFuelAmountMb")
          local okC, cap = pcall(peripheral.call, t, "getFuelCapacityMb")
          if not (okA and okC and type(amount) == "number") then return end
          local space = math.min(cap - amount, spare)
          spare = spare - space -- claimed before the calls, so parallel jobs share it
          -- take from the fullest source first, then the next, until full
          for _, s in ipairs(sources) do
            if space <= 0 then break end
            local ok, moved = pcall(peripheral.call, t, "pullFluid", s.name, space, FUEL)
            if ok and type(moved) == "number" then space = space - moved end
          end
          spare = spare + math.max(space, 0)
        end
      end
      if #jobs > 0 then parallel.waitForAll(table.unpack(jobs)) end
    end

    -- running out: land while there is still fuel to do it
    local flying = F.mode ~= "off" and F.mode ~= "land"
    local low = false
    if F.tankMb then
      low = F.tankMb < LOW_FUEL_MB
    elseif F.st and F.st.thrusters then
      for _, t in ipairs(F.st.thrusters) do
        if t.direction.y > 0.7 and t.fuel and t.fuel < LOW_THRUSTER_MB then low = true end
      end
    end
    F.lowFuel = low
    if low and flying then land("low fuel - landing") end
    sleep(0.5)
  end
end

-- ============================================================ screen + keys

local function draw()
  local W = term.getSize()
  local cx, cy = term.getCursorPos()
  local function line(y, text, colour)
    term.setCursorPos(1, y)
    term.clearLine()
    if colour and term.isColour() then term.setTextColour(colour) end
    term.write(text:sub(1, W))
    term.setTextColour(colours.white)
  end
  local s = F.st
  line(1, "Lava VTOL " .. VERSION .. " - " .. F.mode:upper(), F.mode == "off" and colours.grey or colours.orange)
  if s and s.onShip then
    local v = F.vel or v3()
    line(2, string.format("Height %.1f   ground %s", s.position.y,
      s.heightAboveGround and string.format("%.1f", s.heightAboveGround) or "?"))
    line(3, string.format("Speed %.1f   climb %+.1f   tilt %.1f", len(v), v.y, s.tilt),
      s.tilt > 25 and colours.red or nil)
    line(4, string.format("Lift %d   rear %d at %d%%   hover %s", F.liftN, F.rearN, math.floor(F.rear * 100 + 0.5),
      s.hoverThrottle and string.format("%d%%", math.floor(s.hoverThrottle * 100 + 0.5)) or "?"),
      (s.hoverThrottle or 0) > 0.85 and colours.red or nil)
    local fuel = "Fuel: no tank or basin on network"
    if F.tankMb then
      fuel = string.format("Fuel %d mB in %d block(s)", math.floor(F.tankMb), F.sourceN or 0)
    end
    if (F.basinN or 0) > 0 then
      fuel = fuel .. "   basins " .. F.basinN
      if (F.generatorN or 0) == 0 then fuel = fuel .. " (no cobble gen)" end
    end
    line(5, fuel, F.lowFuel and colours.red or nil)
    -- burners on their own line: which get diesel, which get the other fuel
    local burn = "Burners: none on network"
    local warn = false
    if (F.burnerN or 0) > 0 then
      local dIds, oIds = {}, {}
      for _, b in ipairs(F.burnerList or {}) do
        if isDieselBurner(b) then dIds[#dIds + 1] = burnerNumber(b) else oIds[#oIds + 1] = burnerNumber(b) end
      end
      if #dIds == 0 then
        burn = "Burners " .. table.concat(oIds, " ") .. ": press B to pick the diesel one"
        warn = true
      else
        burn = string.format("Diesel %s (%d mB)", table.concat(dIds, " "), math.floor(F.burnerFuelMb or 0))
        if #oIds > 0 then
          local short = OTHER_BURNER_FUEL and (OTHER_BURNER_FUEL:match(":(.+)$") or OTHER_BURNER_FUEL) or "none"
          burn = burn .. "  " .. short .. " " .. table.concat(oIds, " ")
        end
        warn = (F.burnerFuelMb or 0) == 0
      end
    end
    line(10, burn, warn and colours.red or nil)
    line(6, F.target and string.format("Target %.0f %.0f %.0f", F.target.x, F.target.y, F.target.z) or "Target -")
    if F.mode == "manual" then
      line(7, string.format("Manual: forward %d  sideways %d", F.vF, F.vS), colours.cyan)
    else
      line(7, "")
    end
    local un = s.unwiredThrusterCount or 0
    line(8, un > 0 and (un .. " thruster(s) not wired to the Flight Controller") or "", colours.red)
  else
    line(2, "Not on a ship (assemble it)", colours.red)
    for y = 3, 8 do line(y, "") end
    line(10, "")
  end
  line(9, F.msg, colours.yellow)
  line(11, "T takeoff G goto H hover L land", colours.lightGrey)
  line(12, "M manual B burners O off Q quit", colours.lightGrey)
  term.setCursorPos(cx, cy)
end

local function parseTarget(text)
  local p = here()
  if not p then return nil end
  local words = {}
  for w in text:gmatch("%S+") do words[#words + 1] = w end
  if #words ~= 2 and #words ~= 3 then return nil end
  local base = #words == 2 and { p.x, p.z } or { p.x, p.y, p.z }
  local out = {}
  for i, w in ipairs(words) do
    local rel = w:sub(1, 1) == "~"
    local n = tonumber(rel and (w:sub(2) == "" and "0" or w:sub(2)) or w)
    if not n then return nil end
    out[i] = rel and base[i] + n or n
  end
  if #out == 2 then return out[1], nil, out[2] end
  return out[1], out[2], out[3]
end

local function keyLoop()
  while F.running do
    local _, ch = os.pullEvent("char")
    ch = ch:lower()
    if ch == "t" then takeOff()
    elseif ch == "h" then hoverHere()
    elseif ch == "l" then land()
    elseif ch == "m" then manual()
    elseif ch == "o" then setMode("off", "off")
    elseif ch == "b" and F.mode ~= "manual" then
      F.picking = true -- the flight loop stops drawing over the list
      pcall(pickDieselBurners)
      F.picking = false
      term.clear()
    elseif ch == "q" then setMode("off", "quit") F.running = false
    elseif ch == "g" then
      term.setCursorPos(1, 13)
      term.clearLine()
      term.write("go to x z (or x y z): ")
      local x, y, z = parseTarget(read() or "")
      term.setCursorPos(1, 13)
      term.clearLine()
      if x then goTo(x, y, z) else F.msg = "need 2 or 3 numbers, ~ = relative" end
    elseif F.mode == "manual" then
      if ch == "w" then F.vF = math.min(F.vF + MANUAL_STEP, CRUISE_SPEED)
      elseif ch == "s" then F.vF = math.max(F.vF - MANUAL_STEP, -6)
      elseif ch == "d" then F.vS = math.min(F.vS + MANUAL_STEP, 8)
      elseif ch == "a" then F.vS = math.max(F.vS - MANUAL_STEP, -8)
      elseif ch == "r" then F.target.y = F.target.y + 5
      elseif ch == "f" then F.target.y = F.target.y - 5
      elseif ch == " " then F.vF, F.vS = 0, 0 end
    end
    draw()
  end
end

local function flightLoop()
  local n = 0
  while F.running do
    local _, s = os.pullEvent("flight_state")
    if type(s) == "table" then
      local ok, err = pcall(step, s)
      if not ok then F.msg = "error: " .. tostring(err) end
      n = n + 1
      if n % 4 == 0 and not F.picking then draw() end
    end
  end
end

-- Only update on the ground with the thrusters off.
local function readyToUpdate()
  return F.mode == "off"
end

-- ============================================================ auto-update

-- Every UPDATE_CHECK_MINUTES this checks GitHub for a newer version. A new
-- version is downloaded, checked that it loads, given your current settings
-- (the UPPERCASE lines at the top of this file), saved, and the computer
-- reboots into it. Set AUTO_UPDATE = false at the top to turn this off.
local UPDATE_REPO = "levisnakes/cc-lava-vtol"
local UPDATE_FILE = "vtol.lua"
local UPDATE_CHECK_MINUTES = 15
local PROGRAM_PATH = (shell and shell.getRunningProgram()) or "startup.lua"
local SHA_FILE = "/.version_" .. UPDATE_FILE

-- Settings are the top-of-file assignments to UPPERCASE names, before the
-- first "local" line or "-- ====" section. Tables can span several lines.
local function splitSettings(src)
  local settings, rest = {}, {}
  local inSettings = true
  for line in (src .. "\n"):gmatch("(.-)\n") do
    if inSettings and (line:match("^local ") or line:match("^%-%- ====")) then inSettings = false end
    table.insert(inSettings and settings or rest, line)
  end
  return settings, rest
end

local function braceDelta(line)
  local code = line:gsub('"[^"]*"', ""):gsub("%-%-.*$", "")
  local _, opens = code:gsub("{", "")
  local _, closes = code:gsub("}", "")
  return opens - closes
end

-- [NAME] = the full statement text, possibly several lines
local function statements(lines)
  local out = {}
  local i = 1
  while i <= #lines do
    local name = lines[i]:match("^([%u_][%u%d_]*)%s*=")
    if name then
      local block = { lines[i] }
      local depth = braceDelta(lines[i])
      while depth > 0 and i < #lines do
        i = i + 1
        table.insert(block, lines[i])
        depth = depth + braceDelta(lines[i])
      end
      out[name] = table.concat(block, "\n")
    end
    i = i + 1
  end
  return out
end

-- New file, with each setting that already exists replaced by your value.
local function mergeSettings(oldSrc, newSrc)
  local oldSettings = statements((splitSettings(oldSrc)))
  local newSettings, rest = splitSettings(newSrc)
  local out = {}
  local i = 1
  while i <= #newSettings do
    local line = newSettings[i]
    local name = line:match("^([%u_][%u%d_]*)%s*=")
    if name and oldSettings[name] then
      table.insert(out, oldSettings[name])
      local depth = braceDelta(line)
      while depth > 0 and i < #newSettings do
        i = i + 1
        depth = depth + braceDelta(newSettings[i])
      end
    else
      table.insert(out, line)
    end
    i = i + 1
  end
  for _, line in ipairs(rest) do table.insert(out, line) end
  local merged = table.concat(out, "\n")
  -- splitting added one trailing newline's worth of empty line; match the source
  if newSrc:sub(-1) == "\n" and merged:sub(-1) ~= "\n" then merged = merged .. "\n" end
  return (merged:gsub("\n\n$", "\n"))
end

local function fetch(url, headers)
  local ok, h = pcall(http.get, url, headers)
  if not ok or not h then return nil end
  local body = h.readAll()
  h.close()
  return body
end

local function readText(path)
  if not fs.exists(path) then return nil end
  local f = fs.open(path, "r")
  local t = f.readAll()
  f.close()
  return t
end

local function writeText(path, text)
  local f = fs.open(path, "w")
  f.write(text)
  f.close()
end

local function checkForUpdate(atBoot)
  if AUTO_UPDATE == false or not http then return end
  local sha = fetch("https://api.github.com/repos/" .. UPDATE_REPO .. "/commits/main",
    { ["Accept"] = "application/vnd.github.sha", ["User-Agent"] = "cc-auto-update" })
  if not sha then return end
  sha = sha:match("^%s*(%x+)%s*$")
  if not sha or #sha < 40 or readText(SHA_FILE) == sha then return end

  local newSrc = fetch("https://raw.githubusercontent.com/" .. UPDATE_REPO .. "/" .. sha .. "/" .. UPDATE_FILE)
  if not newSrc or not load(newSrc, UPDATE_FILE) then return end
  local oldSrc = readText(PROGRAM_PATH)
  if not oldSrc then return end
  local merged = mergeSettings(oldSrc, newSrc)
  if not load(merged, UPDATE_FILE) then return end
  if merged == oldSrc then
    writeText(SHA_FILE, sha)
    return
  end

  -- at boot nothing is running yet, so there is nothing to wait for
  while not atBoot and not readyToUpdate() do sleep(5) end
  writeText(PROGRAM_PATH .. ".old", oldSrc)
  writeText(PROGRAM_PATH, merged)
  writeText(SHA_FILE, sha)
  term.setBackgroundColor(colors.black)
  term.clear()
  term.setCursorPos(1, 1)
  print("Updated to " .. sha:sub(1, 7) .. ", restarting...")
  if not atBoot then sleep(1) end
  os.reboot()
end

local function updateLoop()
  while true do
    sleep(UPDATE_CHECK_MINUTES * 60)
    pcall(checkForUpdate)
  end
end

-- Check for a new version as soon as the computer starts, before the
-- program runs, so a restart always picks up the latest version.
local function bootUpdate()
  if AUTO_UPDATE == false or not http then return end
  term.setBackgroundColor(colors.black)
  term.clear()
  term.setCursorPos(1, 1)
  print("Checking for updates...")
  pcall(checkForUpdate, true)
end

-- ============================================================ main

bootUpdate()
if LOG then logf = fs.open("vtol.log", "w") end
term.setBackgroundColour(colours.black)
term.clear()
draw()

-- whatever happens (error, Ctrl+T, Q), the thrusters are cut right away
local okRun, runErr = pcall(parallel.waitForAny, flightLoop, keyLoop, fuelLoop, updateLoop)
pcall(fc.setAllThrust, 0)
if logf then logf.close() end
term.clear()
term.setCursorPos(1, 1)
print("Lava VTOL: thrusters off.")
if not okRun then printError(runErr) end
