local panel
local hideTimer = "navfast_ui_hide"

local function position(frame)
    frame:SetPos(ScrW() - frame:GetWide() - 24, ScrH() - frame:GetTall() - 24)
end

local function createPanel()
    local frame = vgui.Create("DFrame")
    frame:SetSize(290, 133)
    frame:SetDraggable(false)
    frame:ShowCloseButton(false)
    frame:SetKeyboardInputEnabled(false)
    frame:SetMouseInputEnabled(true)
    position(frame)

    frame.phase = frame:Add("DLabel")
    frame.phase:Dock(TOP)
    frame.phase:SetTall(22)

    frame.bar = frame:Add("DProgress")
    frame.bar:Dock(TOP)
    frame.bar:SetTall(12)

    frame.elapsed = frame:Add("DLabel")
    frame.elapsed:Dock(TOP)
    frame.elapsed:SetTall(25)

    frame.cancel = frame:Add("DButton")
    frame.cancel:Dock(RIGHT)
    frame.cancel:SetWide(74)
    frame.cancel:SetText("Cancel")
    frame.cancel.DoClick = function()
        RunConsoleCommand("nav_generate_fast_cancel")
        frame.cancel:SetEnabled(false)
    end

    frame.Think = function(self)
        local ply = LocalPlayer()
        if not IsValid(ply) or not ply:IsSuperAdmin() then self:Remove() end
    end

    return frame
end

hook.Add("OnScreenSizeChanged", "navfast_ui_position", function()
    if IsValid(panel) then position(panel) end
end)

net.Receive("navfast_progress", function()
    local map = net.ReadString()
    local phase = net.ReadString()
    local progress = net.ReadFloat()
    local elapsed = net.ReadFloat()
    local done = net.ReadBool()
    local message = net.ReadString()
    local ply = LocalPlayer()
    if not IsValid(ply) or not ply:IsSuperAdmin() then
        if IsValid(panel) then panel:Remove() end
        return
    end
    if done and not IsValid(panel) then return end
    if not IsValid(panel) then panel = createPanel() end

    panel:SetTitle("Navigation  ·  " .. map)
    panel.phase:SetText(phase)
    panel.phase:SetTooltip(message)
    panel.bar:SetFraction(progress)
    panel.elapsed:SetText(string.format("%.1f s", elapsed))
    panel.cancel:SetVisible(not done)
    if done then
        timer.Create(hideTimer, 3, 1, function()
            if IsValid(panel) then panel:Remove() end
        end)
    else
        if timer.Exists(hideTimer) then panel.cancel:SetEnabled(true) end
        timer.Remove(hideTimer)
    end
end)
