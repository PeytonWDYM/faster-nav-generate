-- Streams the loaded nav mesh to superadmins running the client-side viewer (`nav_view_fast 1`).
-- Works with any loaded mesh (vanilla or navfast) and does not need the binary module.
AddCSLuaFile("autorun/client/navfast_viewer.lua")
AddCSLuaFile("navfast/viewer_mesh.lua")

local requestMessage = "navfast_view_request"
local dataMessage = "navfast_view_data"
util.AddNetworkString(requestMessage)
util.AddNetworkString(dataMessage)

-- Message kinds on navfast_view_data.
local BEGIN, CHUNK, FINISH = 0, 1, 2
-- ~80 bytes per area keeps each chunk well under the 64 KB net message limit.
local areasPerChunk = 400

local viewers = {}   -- [Player] = true while their viewer is on
local transfers = {} -- [Player] = { serial, areas, index }
local serial = 0

-- Snapshot every area as plain data so the transfer is immune to the mesh changing mid-send.
local function snapshotAreas()
    local areas = {}
    for i, area in ipairs(navmesh.GetAllNavAreas()) do
        local adjacent = {}
        for j, other in ipairs(area:GetAdjacentAreas()) do adjacent[j] = other:GetID() end
        areas[i] = {
            id = area:GetID(),
            attributes = area:GetAttributes(),
            nw = area:GetCorner(0),
            ne = area:GetCorner(1),
            se = area:GetCorner(2),
            sw = area:GetCorner(3),
            adjacent = adjacent
        }
    end
    return areas
end

local function writeVector(v)
    net.WriteFloat(v.x)
    net.WriteFloat(v.y)
    net.WriteFloat(v.z)
end

local function startTransfer(ply)
    serial = serial + 1
    local areas = snapshotAreas()
    transfers[ply] = {serial = serial, areas = areas, index = 1}
    net.Start(dataMessage)
    net.WriteUInt(BEGIN, 2)
    net.WriteUInt(serial, 32)
    net.WriteString(game.GetMap())
    net.WriteUInt(#areas, 32)
    net.Send(ply)
end

-- Sends one chunk per player per tick so large meshes never flood the reliable channel.
local function pumpTransfers()
    for ply, transfer in pairs(transfers) do
        if not IsValid(ply) then
            transfers[ply] = nil
        else
            local areas = transfer.areas
            local last = math.min(transfer.index + areasPerChunk - 1, #areas)
            net.Start(dataMessage)
            if transfer.index > #areas then
                net.WriteUInt(FINISH, 2)
                net.WriteUInt(transfer.serial, 32)
                transfers[ply] = nil
            else
                net.WriteUInt(CHUNK, 2)
                net.WriteUInt(transfer.serial, 32)
                net.WriteUInt(last - transfer.index + 1, 16)
                for i = transfer.index, last do
                    local area = areas[i]
                    net.WriteUInt(area.id, 32)
                    net.WriteUInt(area.attributes, 32)
                    writeVector(area.nw)
                    writeVector(area.se)
                    net.WriteFloat(area.ne.z)
                    net.WriteFloat(area.sw.z)
                    net.WriteUInt(#area.adjacent, 16)
                    for _, id in ipairs(area.adjacent) do net.WriteUInt(id, 32) end
                end
                transfer.index = last + 1
            end
            net.Send(ply)
        end
    end
end

net.Receive(requestMessage, function(_, ply)
    if not ply:IsSuperAdmin() then return end
    if net.ReadBool() then
        viewers[ply] = true
        startTransfer(ply)
    else
        viewers[ply] = nil
        transfers[ply] = nil
    end
end)

-- Fired by navfast.lua after nav_generate_fast loads a new mesh: refresh every open viewer.
hook.Add("NavFastMeshLoaded", "navfast_viewer", function()
    for ply in pairs(viewers) do
        if IsValid(ply) then startTransfer(ply) else viewers[ply] = nil end
    end
end)

hook.Add("PlayerDisconnected", "navfast_viewer", function(ply)
    viewers[ply] = nil
    transfers[ply] = nil
end)

hook.Add("Tick", "navfast_viewer", function()
    if next(transfers) then pumpTransfers() end
end)
