AddCSLuaFile("autorun/client/navfast_ui.lua")

if not util.IsBinaryModuleInstalled("navfast") then
    print("[navfast] Binary module is missing. Install gmsv_navfast for this server's platform and architecture.")
    return
end

require("navfast")

local makeVerifier = include("navfast/verify.lua")
local workerThreads = CreateConVar("nav_generate_fast_threads", "0", FCVAR_ARCHIVE, "Worker threads. 0 uses all logical cores.", 0)
local dumpEnabled = CreateConVar("nav_generate_fast_dump", "0", FCVAR_ARCHIVE, "Save generation inputs for offline replay.", 0, 1)
local progressMessage = "navfast_progress"
local pollTimer = "navfast_poll"
util.AddNetworkString(progressMessage)

local spawnClasses = {
    "info_player_start", "info_player_deathmatch", "info_player_combine", "info_player_rebel",
    "info_player_counterterrorist", "info_player_terrorist", "gmod_player_start"
}
local tuning = {
    slope_limit = "GetFloat",
    slope_tolerance = "GetFloat",
    displacement_test = "GetInt",
    area_max_size = "GetInt",
    generate_fencetops = "GetBool",
    generate_fixup_jump_areas = "GetBool",
    generate_jump_connections = "GetBool"
}
local active
local lastNetwork = -math.huge

local function log(message)
    print("[navfast] " .. message)
end

local function allowed(ply)
    return not IsValid(ply) or ply:IsSuperAdmin()
end

local function spawnPositions()
    local seeds, seen = {}, {}
    local function add(class)
        for _, ent in ipairs(ents.FindByClass(class)) do
            if not seen[ent] then
                seen[ent] = true
                seeds[#seeds + 1] = ent
            end
        end
    end
    for _, class in ipairs(spawnClasses) do add(class) end
    add(navmesh.GetPlayerSpawnName())
    table.sort(seeds, function(a, b) return a:EntIndex() < b:EntIndex() end)
    for i, ent in ipairs(seeds) do seeds[i] = ent:GetPos() end
    return seeds
end

local function blocksNavigation(ent)
    if ent:IsWorld() or ent:IsPlayer() or ent:IsNPC() or ent:IsNextBot() or ent:IsWeapon() or not ent:IsSolid() then
        return false
    end
    local group = ent:GetCollisionGroup()
    if group == COLLISION_GROUP_DEBRIS or group == COLLISION_GROUP_DEBRIS_TRIGGER then return false end
    local class = ent:GetClass()
    if string.StartWith(class, "func_door") or string.StartWith(class, "prop_door") then return false end
    if class == "func_breakable" and ent:Health() > 0 then return false end
    if class == "func_breakable_surf" and ent:GetInternalVariable("m_takedamage") == 2 then return false end
    if class == "func_playerinfected_clip" then return false end
    if class == "func_brush" then
        local solidity = tonumber(ent:GetInternalVariable("m_iSolidity") or ent:GetKeyValues().Solidity)
        return solidity == 2 -- BRUSHSOLID_ALWAYS
    end
    return true
end

local function loadCollisionInputs(map, worldOnly)
    local bsp = file.Read("maps/" .. map .. ".bsp", "GAME")
    if not bsp then error("Unable to read maps/" .. map .. ".bsp from GAME", 0) end
    local phyFiles, seen, solids = {}, {}, {}
    local function loadPhy(model)
        if seen[model] then return end
        seen[model] = true
        phyFiles[model] = file.Read((model:gsub("%.mdl$", ".phy")), "GAME")
    end
    for _, model in ipairs(navfast.StaticPropModels(bsp)) do loadPhy(model) end
    if not worldOnly then
        for _, ent in ipairs(ents.GetAll()) do
            if blocksNavigation(ent) then
                local model = ent:GetModel()
                if model then
                    model = model:lower():gsub("\\", "/")
                    if model:match("^%*%d+$") or model:match("%.mdl$") then
                        solids[#solids + 1] = {model = model, pos = ent:GetPos(), ang = ent:GetAngles()}
                        if model:match("%.mdl$") then loadPhy(model) end
                    end
                end
            end
        end
    end
    return bsp, phyFiles, solids
end

local function gameDirectory()
    if util.RelativePathToFull then
        local path = util.RelativePathToFull("gameinfo.txt")
        if path and path ~= "" then return path:gsub("[/\\]gameinfo%.txt$", "") end
    end
    -- Current GMod launches from the directory that contains garrysmod.
    return "garrysmod"
end

local function jobParams(map, worldOnly)
    local params = {threads = workerThreads:GetInt(), mode = worldOnly and "world" or "generate"}
    if not worldOnly then
        local directory = gameDirectory()
        params.outputPath = directory .. "/maps/" .. map .. ".nav"
        if dumpEnabled:GetBool() then
            file.CreateDir("navfast")
            params.dumpPath = directory .. "/data/navfast/" .. map .. ".navjob"
        end
    end
    for key, getter in pairs(tuning) do
        local convar = GetConVar("nav_" .. key)
        if convar then params[key] = convar[getter](convar) end
    end
    return params
end

local function sendProgress(job, phase, fraction, message, done)
    local now = SysTime()
    if now - lastNetwork < 0.2 then return false end
    lastNetwork = now
    local admins = {}
    for _, ply in ipairs(player.GetHumans()) do
        if ply:IsSuperAdmin() then admins[#admins + 1] = ply end
    end
    if #admins > 0 then
        net.Start(progressMessage)
        net.WriteString(job.map)
        net.WriteString(phase)
        net.WriteFloat(fraction)
        net.WriteFloat(now - job.started)
        net.WriteBool(done)
        net.WriteString(message or "")
        net.Send(admins)
    end
    return true
end

local function reportProgress(job, phase, progress, message)
    local percent = math.floor(progress * 10) * 10
    if job.phase ~= phase or percent > job.percent then
        local suffix = message and message ~= "" and (" (" .. message .. ")") or ""
        log(string.format("%s %d%%%s", phase, percent, suffix))
        job.phase, job.percent = phase, percent
    end
    sendProgress(job, phase, progress, message, false)
end

local function finish(job, phase, message)
    log(message)
    job.finishedPhase, job.finishedMessage = phase, message
end

local function poll()
    local job = active
    if not job.finishedPhase then
        if job.verify then
            if job.cancelled then
                finish(job, "Cancelled", "Verification cancelled.")
            else
                local ok, fraction, message = pcall(job.verify)
                if not ok then finish(job, "Error", tostring(fraction))
                elseif fraction == 1 then finish(job, "Done", message)
                else reportProgress(job, "VerifyTraces", fraction, message) end
            end
        else
            local status = navfast.Poll()
            if status.running then
                reportProgress(job, status.phase, status.progress, status.message)
            elseif status.done then
                if status.error then
                    finish(job, status.error == "Cancelled" and "Cancelled" or "Error", status.error)
                elseif job.verifyCount then
                    job.verify = makeVerifier(job.map, job.seeds, job.verifyCount, status.stats.worldMins, status.stats.worldMaxs)
                else
                    navmesh.Load()
                    finish(job, "Done", string.format("Loaded %d areas in %.2f seconds.", status.stats.areas, status.stats.seconds))
                end
            end
        end
    end
    if job.finishedPhase and sendProgress(job, job.finishedPhase, job.finishedPhase == "Done" and 1 or 0,
        job.finishedMessage, true) then
        timer.Remove(pollTimer)
        active = nil
    end
end

local function start(verifyCount)
    if active or navfast.Poll().running then
        log("A job is already running.")
        return
    end
    local map = game.GetMap()
    local started = SysTime()
    local worldOnly = verifyCount ~= nil
    local seeds = spawnPositions()
    local ok, message = pcall(function()
        local bsp, phyFiles, solids = loadCollisionInputs(map, worldOnly)
        navfast.Start(bsp, phyFiles, solids, seeds, jobParams(map, worldOnly))
    end)
    if not ok then
        log(tostring(message))
        return
    end
    active = {map = map, started = started, seeds = seeds, percent = -1, verifyCount = verifyCount}
    timer.Create(pollTimer, 0.1, 0, poll)
end

concommand.Add("nav_generate_fast", function(ply)
    if allowed(ply) then start() end
end)

concommand.Add("nav_generate_fast_cancel", function(ply)
    if not allowed(ply) or not active or active.finishedPhase then return end
    active.cancelled = true
    navfast.Cancel()
end)

concommand.Add("nav_generate_fast_verify", function(ply, _, args)
    if not allowed(ply) then return end
    local count = tonumber(args[1] or "20000")
    if not count or count < 1 or count == math.huge or count ~= math.floor(count) then
        log("Usage: nav_generate_fast_verify [positive trace count]")
        return
    end
    start(count)
end)
