-- Client-side nav mesh model for the viewer: area geometry, static IMesh batches, picking.
-- Visual language follows Source's nav_edit (nav_area.cpp CNavArea::Draw, nav_colors.cpp).

local CROUCH, JUMP, PRECISE, NO_JUMP, STAIRS = 0x1, 0x2, 0x4, 0x8, 0x1000
local ATTRIBUTE_NAMES = {
    {0x1, "CROUCH"}, {0x2, "JUMP"}, {0x4, "PRECISE"}, {0x8, "NO_JUMP"}, {0x10, "STOP"},
    {0x20, "RUN"}, {0x40, "WALK"}, {0x80, "AVOID"}, {0x100, "TRANSIENT"}, {0x200, "DONT_HIDE"},
    {0x400, "STAND"}, {0x800, "NO_HOSTAGES"}, {0x1000, "STAIRS"}, {0x2000, "NO_MERGE"},
    {0x4000, "OBSTACLE_TOP"}, {0x8000, "CLIFF"}
}

local colors = {
    normal = Color(255, 0, 0),
    selected = Color(255, 255, 0),
    crouch = Color(0, 0, 255),
    jump = Color(0, 255, 0),
    precise = Color(0, 255, 0),
    noJump = Color(255, 0, 0),
    stairs = Color(0, 200, 0),
    twoWay = Color(0, 255, 255),
    oneWay = Color(0, 0, 255),
    cursor = Color(255, 255, 255),
    fill = Color(0, 0, 0, 30) -- nav_area_bgcolor default
}

local CELL = 512           -- spatial bucket size (units)
local MAX_VERTS = 30000    -- stay under IMesh's per-batch vertex limit
local LIFT = Vector(0, 0, 1)
local STEP_HEIGHT = 18

local M = {colors = colors}

local function cellKey(cx, cy) return cx .. ":" .. cy end
local function cellOf(v) return math.floor(v / CELL) end

-- Height of the area's surface at (x, y), as CNavArea::GetZ.
function M.areaZ(area, x, y)
    local nw, se = area.nw, area.se
    local u = (x - nw.x) / math.max(se.x - nw.x, 1)
    local v = (y - nw.y) / math.max(se.y - nw.y, 1)
    local northZ = nw.z + u * (area.ne.z - nw.z)
    local southZ = area.sw.z + u * (se.z - area.sw.z)
    return northZ + v * (southZ - northZ)
end

function M.attributeText(attributes)
    local names = {}
    for _, entry in ipairs(ATTRIBUTE_NAMES) do
        if bit.band(attributes, entry[1]) ~= 0 then names[#names + 1] = entry[2] end
    end
    return #names > 0 and table.concat(names, " ") or "none"
end

-- Builds a viewer area from the wire fields (nw/se corners plus the ne/sw heights).
function M.newArea(id, attributes, nw, se, neZ, swZ, adjacent)
    local area = {
        id = id, attributes = attributes, adjacent = adjacent,
        nw = nw, se = se, ne = Vector(se.x, nw.y, neZ), sw = Vector(nw.x, se.y, swZ)
    }
    area.center = (area.nw + area.ne + area.se + area.sw) * 0.25
    return area
end

-- Outline corners inset 0.2 units like nav_edit, lifted to avoid z-fighting.
function M.outline(area, lift)
    local up = lift or LIFT
    local i = 0.2
    return Vector(area.nw.x + i, area.nw.y + i, area.nw.z) + up,
        Vector(area.ne.x - i, area.ne.y + i, area.ne.z) + up,
        Vector(area.se.x - i, area.se.y - i, area.se.z) + up,
        Vector(area.sw.x + i, area.sw.y - i, area.sw.z) + up
end

-- Calls line(a, b, color) for the outline and attribute markings of one area.
local function emitAreaLines(area, line)
    local nw, ne, se, sw = M.outline(area)
    line(nw, ne, colors.normal)
    line(ne, se, colors.normal)
    line(se, sw, colors.normal)
    line(sw, nw, colors.normal)

    local attributes = area.attributes
    if bit.band(attributes, CROUCH) ~= 0 then line(nw, se, colors.crouch) end
    if bit.band(attributes, JUMP) ~= 0 then
        if bit.band(attributes, CROUCH) == 0 then line(nw, se, colors.jump) end
        line(ne, sw, colors.jump)
    end

    local c, size = area.center + LIFT, 8
    if bit.band(attributes, PRECISE) ~= 0 then
        line(c + Vector(0, -size, 0), c + Vector(0, size, 0), colors.precise)
        line(c + Vector(-size, 0, 0), c + Vector(size, 0, 0), colors.precise)
    end
    if bit.band(attributes, NO_JUMP) ~= 0 then
        local up, right, down, left = c + Vector(0, -size, 0), c + Vector(size, 0, 0), c + Vector(0, size, 0), c + Vector(-size, 0, 0)
        line(up, right, colors.noJump)
        line(right, down, colors.noJump)
        line(down, left, colors.noJump)
        line(left, up, colors.noJump)
    end
    if bit.band(attributes, STAIRS) ~= 0 then
        -- Stripes every half step across the axis with the larger height change.
        local eastWest = math.abs((area.nw.z + area.sw.z) - (area.ne.z + area.se.z))
        local northSouth = math.abs((area.nw.z + area.ne.z) - (area.sw.z + area.se.z))
        local step = STEP_HEIGHT / 2
        if eastWest > northSouth then
            for x = area.nw.x, area.se.x, step do
                line(Vector(x, area.nw.y, M.areaZ(area, x, area.nw.y)) + LIFT,
                    Vector(x, area.se.y, M.areaZ(area, x, area.se.y)) + LIFT, colors.stairs)
            end
        else
            for y = area.nw.y, area.se.y, step do
                line(Vector(area.nw.x, y, M.areaZ(area, area.nw.x, y)) + LIFT,
                    Vector(area.se.x, y, M.areaZ(area, area.se.x, y)) + LIFT, colors.stairs)
            end
        end
    end
end

-- Packs vertex records { pos, color } into as many IMeshes as the vertex limit requires.
local function buildMeshes(primitive, vertsPerPrimitive, verts)
    local meshes = {}
    local perBatch = math.floor(MAX_VERTS / vertsPerPrimitive) * vertsPerPrimitive
    for first = 1, #verts, perBatch do
        local last = math.min(first + perBatch - 1, #verts)
        local batch = Mesh()
        mesh.Begin(batch, primitive, (last - first + 1) / vertsPerPrimitive)
        for i = first, last do
            local vert = verts[i]
            mesh.Position(vert[1])
            mesh.Color(vert[2].r, vert[2].g, vert[2].b, vert[2].a)
            mesh.AdvanceVertex()
        end
        mesh.End()
        meshes[#meshes + 1] = batch
    end
    return meshes
end

-- Turns the received area list into draw cells (bucketed by area center) and a pick index
-- (each area listed in every cell its footprint overlaps).
function M.build(areas)
    local byId, drawCells, pickCells = {}, {}, {}
    local lineVerts, fillVerts = {}, {}

    for _, area in ipairs(areas) do
        byId[area.id] = area
        local key = cellKey(cellOf(area.center.x), cellOf(area.center.y))
        local cell = drawCells[key]
        if not cell then
            cell = {
                center = Vector((cellOf(area.center.x) + 0.5) * CELL, (cellOf(area.center.y) + 0.5) * CELL, area.center.z),
                areas = {}
            }
            drawCells[key] = cell
            lineVerts[key], fillVerts[key] = {}, {}
        end
        cell.areas[#cell.areas + 1] = area

        local lines = lineVerts[key]
        emitAreaLines(area, function(a, b, color)
            lines[#lines + 1] = {a, color}
            lines[#lines + 1] = {b, color}
        end)
        local fill = fillVerts[key]
        local lift = Vector(0, 0, 0.8)
        -- Both windings, so backface culling never hides the fill whichever way it faces.
        for _, corner in ipairs({area.nw, area.ne, area.se, area.sw, area.sw, area.se, area.ne, area.nw}) do
            fill[#fill + 1] = {corner + lift, colors.fill}
        end

        for cx = cellOf(area.nw.x), cellOf(area.se.x) do
            for cy = cellOf(area.nw.y), cellOf(area.se.y) do
                local pickKey = cellKey(cx, cy)
                pickCells[pickKey] = pickCells[pickKey] or {}
                table.insert(pickCells[pickKey], area)
            end
        end
    end

    for key, cell in pairs(drawCells) do
        cell.lines = buildMeshes(MATERIAL_LINES, 2, lineVerts[key])
        cell.fill = buildMeshes(MATERIAL_QUADS, 4, fillVerts[key])
    end
    return {areas = areas, byId = byId, drawCells = drawCells, pickCells = pickCells}
end

function M.destroy(model)
    for _, cell in pairs(model.drawCells) do
        for _, batch in ipairs(cell.lines) do batch:Destroy() end
        for _, batch in ipairs(cell.fill) do batch:Destroy() end
    end
end

-- Area whose surface is closest to `pos` (within a step) among areas covering pos.x/pos.y.
function M.pick(model, pos)
    local best, bestDelta = nil, STEP_HEIGHT * 2
    for _, area in ipairs(model.pickCells[cellKey(cellOf(pos.x), cellOf(pos.y))] or {}) do
        if pos.x >= area.nw.x and pos.x <= area.se.x and pos.y >= area.nw.y and pos.y <= area.se.y then
            local delta = math.abs(M.areaZ(area, pos.x, pos.y) - pos.z)
            if delta < bestDelta then best, bestDelta = area, delta end
        end
    end
    return best
end

-- Draw cells whose bucket lies within `range` of `eye` (padded by a bucket so large areas near
-- the edge still show).
function M.visibleCells(model, eye, range)
    local padded = (range + CELL) ^ 2
    local cells = {}
    for _, cell in pairs(model.drawCells) do
        if cell.center:DistToSqr(eye) <= padded then cells[#cells + 1] = cell end
    end
    return cells
end

return M
