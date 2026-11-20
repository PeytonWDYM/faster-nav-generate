local playerMins = Vector(-16, -16, 0)
local playerMaxs = Vector(16, 16, 72)
local zero = Vector(0, 0, 0)
local function ignoreEntities() return false end

local function randomPoint(mins, maxs)
    return Vector(math.Rand(mins.x, maxs.x), math.Rand(mins.y, maxs.y), math.Rand(mins.z, maxs.z))
end

local function xyz(vec)
    return {vec.x, vec.y, vec.z}
end

local function traceSample(state)
    local aroundSpawn = state.tested % 4 < 2 and #state.seeds > 0
    local hull = state.tested % 2 == 0
    local start
    if aroundSpawn then
        local seed = state.seeds[math.random(#state.seeds)]
        start = seed + Vector(math.Rand(-192, 192), math.Rand(-192, 192), math.Rand(-64, 192))
    else
        start = randomPoint(state.mins, state.maxs)
    end

    local endpos = start + Vector(math.Rand(-256, 256), math.Rand(-256, 256), math.Rand(-512, 256))
    local mins, maxs = zero, zero
    if hull then mins, maxs = playerMins, playerMaxs end
    local engine = util.TraceHull({
        start = start,
        endpos = endpos,
        mins = mins,
        maxs = maxs,
        mask = MASK_NPCSOLID_BRUSHONLY,
        filter = ignoreEntities
    })
    local fast = navfast.TraceHull(start, endpos, mins, maxs, MASK_NPCSOLID_BRUSHONLY)
    local distanceError = math.abs(engine.Fraction - fast.Fraction) * start:Distance(endpos)
    if fast.StartSolid == engine.StartSolid and distanceError <= 1 then return end

    state.mismatches = state.mismatches + 1
    if #state.samples < 100 then
        state.samples[#state.samples + 1] = {
            start = xyz(start),
            endpos = xyz(endpos),
            hull = hull,
            distanceError = distanceError,
            engine = {fraction = engine.Fraction, startSolid = engine.StartSolid, hitPos = xyz(engine.HitPos)},
            navfast = {fraction = fast.Fraction, startSolid = fast.StartSolid, hitPos = xyz(fast.HitPos)}
        }
    end
end

local function writeReport(state)
    file.CreateDir("navfast")
    local path = "navfast/verify_" .. state.map .. ".json"
    local rate = state.mismatches / state.tested
    file.Write(path, util.TableToJSON({
        map = state.map,
        tested = state.tested,
        mismatches = state.mismatches,
        mismatchRate = rate,
        samples = state.samples
    }, true))
    return string.format("%d / %d trace mismatches (%.2f%%). Report saved to data/%s",
        state.mismatches, state.tested, rate * 100, path)
end

return function(map, seeds, count, mins, maxs)
    local state = {
        map = map,
        seeds = seeds,
        mins = mins,
        maxs = maxs,
        tested = 0,
        mismatches = 0,
        samples = {}
    }
    -- Bound the verification work per server tick.
    return function()
        local untilTime = SysTime() + 0.003
        repeat
            traceSample(state)
            state.tested = state.tested + 1
        until state.tested == count or SysTime() >= untilTime

        if state.tested == count then return 1, writeReport(state) end
        return state.tested / count, string.format("%d / %d traces", state.tested, count)
    end
end
