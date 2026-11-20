-- Client-side nav mesh viewer, modelled on nav_edit 1 but with no sv_cheats and working on
-- dedicated servers. Superadmins only: `nav_view_fast 1` to show, `nav_view_fast 0` to hide.
local viewerMesh = include("navfast/viewer_mesh.lua")
local colors = viewerMesh.colors

local enabled = CreateClientConVar("nav_view_fast", "0", false, false, "Show the server's nav mesh (superadmins).", 0, 1)
local range = CreateClientConVar("nav_view_fast_range", "1500", true, false, "Draw distance for nav_view_fast.", 250, 20000)

local BEGIN, CHUNK, FINISH = 0, 1, 2
local pending -- { serial, map, total, areas } while a transfer is arriving
local model   -- built viewer model, see viewer_mesh.lua
local modelMap
local selected, cursorPos

local function clearModel()
    if model then viewerMesh.destroy(model) end
    model, modelMap, selected, cursorPos = nil, nil, nil, nil
end

local function request(on)
    net.Start("navfast_view_request")
    net.WriteBool(on)
    net.SendToServer()
end

local function readVector()
    return Vector(net.ReadFloat(), net.ReadFloat(), net.ReadFloat())
end

net.Receive("navfast_view_data", function()
    local kind = net.ReadUInt(2)
    local serial = net.ReadUInt(32)
    if kind == BEGIN then
        pending = {serial = serial, map = net.ReadString(), total = net.ReadUInt(32), areas = {}}
        return
    end
    if not pending or pending.serial ~= serial then return end
    if kind == CHUNK then
        for _ = 1, net.ReadUInt(16) do
            local id, attributes = net.ReadUInt(32), net.ReadUInt(32)
            local nw, se = readVector(), readVector()
            local neZ, swZ = net.ReadFloat(), net.ReadFloat()
            local adjacent = {}
            for i = 1, net.ReadUInt(16) do adjacent[i] = net.ReadUInt(32) end
            pending.areas[#pending.areas + 1] = viewerMesh.newArea(id, attributes, nw, se, neZ, swZ, adjacent)
        end
    elseif kind == FINISH then
        clearModel()
        if enabled:GetBool() then
            model, modelMap = viewerMesh.build(pending.areas), pending.map
        end
        pending = nil
    end
end)

cvars.AddChangeCallback("nav_view_fast", function(_, _, value)
    local on = tobool(value)
    if not on then
        clearModel()
        pending = nil
    end
    request(on)
end, "navfast_viewer")

local function isTwoWay(from, to)
    for _, id in ipairs(to.adjacent) do
        if id == from.id then return true end
    end
    return false
end

local function drawOutline(area, color)
    local nw, ne, se, sw = viewerMesh.outline(area, Vector(0, 0, 1.5))
    render.DrawLine(nw, ne, color, true)
    render.DrawLine(ne, se, color, true)
    render.DrawLine(se, sw, color, true)
    render.DrawLine(sw, nw, color, true)
end

-- Selected area in yellow, its neighbours outlined, and a line to each one:
-- cyan for two-way connections, blue for one-way (nav_edit's colours).
local function drawSelection()
    if cursorPos then
        local size = 10
        render.DrawLine(cursorPos, cursorPos + Vector(0, 0, size), colors.cursor, true)
        render.DrawLine(cursorPos - Vector(size, 0, 0), cursorPos + Vector(size, 0, 0), colors.cursor, true)
        render.DrawLine(cursorPos - Vector(0, size, 0), cursorPos + Vector(0, size, 0), colors.cursor, true)
    end
    if not selected then return end
    drawOutline(selected, colors.selected)
    local from = selected.center + Vector(0, 0, 2)
    for _, id in ipairs(selected.adjacent) do
        local other = model.byId[id]
        if other then
            local color = isTwoWay(selected, other) and colors.twoWay or colors.oneWay
            drawOutline(other, color)
            render.DrawLine(from, other.center + Vector(0, 0, 2), color, true)
        end
    end
end

hook.Add("PostDrawTranslucentRenderables", "navfast_viewer", function(_, skybox, skybox3d)
    if not model or skybox or skybox3d then return end
    render.SetColorMaterial()
    for _, cell in ipairs(viewerMesh.visibleCells(model, EyePos(), range:GetFloat())) do
        for _, batch in ipairs(cell.fill) do batch:Draw() end
        for _, batch in ipairs(cell.lines) do batch:Draw() end
    end
    drawSelection()
end)

-- Area under the crosshair, refreshed every tick rather than every frame.
hook.Add("Tick", "navfast_viewer", function()
    if not model then return end
    local ply = LocalPlayer()
    local trace = util.TraceLine({
        start = ply:EyePos(),
        endpos = ply:EyePos() + ply:GetAimVector() * 2000,
        filter = ply,
        mask = MASK_PLAYERSOLID_BRUSHONLY
    })
    cursorPos = trace.Hit and trace.HitPos or nil
    selected = cursorPos and viewerMesh.pick(model, cursorPos) or nil
end)

local function hudText(text, font, y, color)
    draw.SimpleTextOutlined(text, font, ScrW() / 2, y, color, TEXT_ALIGN_CENTER, TEXT_ALIGN_TOP, 1, color_black)
end

hook.Add("HUDPaint", "navfast_viewer", function()
    if not enabled:GetBool() then return end
    local y = ScrH() * 0.12
    if pending then
        local percent = pending.total > 0 and math.floor(#pending.areas / pending.total * 100) or 0
        hudText(string.format("Receiving nav mesh... %d%%", percent), "DermaDefaultBold", y, color_white)
        return
    end
    if not model then
        hudText("Waiting for nav mesh (superadmin only)", "DermaDefaultBold", y, color_white)
        return
    end
    hudText(string.format("%s: %d areas", modelMap, #model.areas), "DermaDefault", y, color_white)
    if selected then
        local width = selected.se.x - selected.nw.x
        local depth = selected.se.y - selected.nw.y
        hudText(string.format("Area #%d", selected.id), "DermaLarge", y + 16, colors.selected)
        hudText(string.format("%.0f x %.0f  |  %d connections  |  %s", width, depth, #selected.adjacent,
            viewerMesh.attributeText(selected.attributes)), "DermaDefaultBold", y + 50, color_white)
    end
end)
