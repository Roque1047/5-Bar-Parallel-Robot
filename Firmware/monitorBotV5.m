clear; clc; close all;
%% ===== UDP Setup =====
localPort     = 5001;
UDPdevicePort = 5000;
u = udpport("datagram", "IPV4", "LocalPort", localPort);
disp("Waiting for device...");

%% ===== Handshake =====
confirmed = false;
while ~confirmed
    if u.NumDatagramsAvailable > 0
        d = read(u, 1, "string");
        if contains(d.Data, "Ready")
            UDPdeviceIP = d.SenderAddress;
            write(u, "ACK", "string", UDPdeviceIP, UDPdevicePort);
            confirmed = true;
        end
    end
    pause(0.05);
end
disp("Connected to device at " + UDPdeviceIP);

%% ===== Geometry =====
p.L1 = 6;
p.L2 = 6;
p.L3 = 15;
p.L4 = 15;
p.L5 = 17;

%% ===== Camera Setup =====
try
    cam = webcam('HD Pro Webcam C920', Resolution='352x288', FocusMode='manual');
catch
    cam = [];
    disp("[warn] Camera not found - camera mode will be unavailable");
end

% Vision calibration
camCal.px_por_cm = 10.05714285714286;
camCal.x0        = 90;
camCal.y0        = 225; % offset image so left motor overlaps
camCal.frameW    = 352;
camCal.frameH    = 288;

% Camera frame bounds mapped to robot coordinate frame (cm)
camCal.xMin =  -(camCal.x0)                 / camCal.px_por_cm;
camCal.xMax =  (camCal.frameW - camCal.x0)  / camCal.px_por_cm;
camCal.yMax =   camCal.y0                   / camCal.px_por_cm;
camCal.yMin =  -(camCal.frameH - camCal.y0) / camCal.px_por_cm;

%% ===== HMI =====
wsPoints = computeWorkspace(p);

fig = figure(Name='5-Bar Monitor', NumberTitle='off');
ax  = axes(Parent=fig);
ax.XAxisLocation = 'top';
axis(ax, 'equal');
xlim(ax, [-p.L1-4, p.L5+p.L2+4]);
ylim(ax, [-p.L1-.5, 20]);
grid(ax, 'on');
ax.XTick = -p.L1-4:1:p.L5+p.L2+4;
ax.YTick = -p.L1-1:1:20;
hold(ax, 'on');
title(ax, "5-Bar Kinematic Monitor");

% camera background
h.camImage         = image(ax, zeros(camCal.frameH, camCal.frameW, 3, 'uint8'));
h.camImage.XData   = [camCal.xMin, camCal.xMax];
h.camImage.YData   = [camCal.yMax, camCal.yMin];
h.camImage.Visible = 'off';
uistack(h.camImage, 'bottom');

% workspace
plot(ax, wsPoints(:,1), wsPoints(:,2), Color='#fd0', LineWidth=1.2);

% base link
plot(ax, [0, p.L5], [0, 0], Color='#754', LineWidth=14);
scatter(ax, [0, p.L5], [0, 0], 500, 'filled', MarkerFaceColor='#fd0');

% expected links
ghostC = '#555';
h.gL1  = plot(ax, [0 0],[0 0], Color=ghostC, LineWidth=3);
h.gL2  = plot(ax, [0 0],[0 0], Color=ghostC, LineWidth=3);
h.gL3  = plot(ax, [0 0],[0 0], Color=ghostC, LineWidth=3);
h.gL4  = plot(ax, [0 0],[0 0], Color=ghostC, LineWidth=3);

% reported links
realC  = '#c72';
joinC  = '#555';
h.rL1  = plot(ax, [0 0],[0 0], Color=realC, LineWidth=8, Marker='o', MarkerEdgeColor=joinC);
h.rL2  = plot(ax, [0 0],[0 0], Color=realC, LineWidth=8, Marker='o', MarkerEdgeColor=joinC);
h.rL3  = plot(ax, [0 0],[0 0], Color=realC, LineWidth=8, Marker='o', MarkerEdgeColor=joinC);
h.rL4  = plot(ax, [0 0],[0 0], Color=realC, LineWidth=8, Marker='o', MarkerEdgeColor=joinC);

% FK traced path
h.fkPath              = plot(ax, NaN, NaN, '-',  Color='#3df', LineWidth=1.5);

% cv traced path
h.visionPath          = plot(ax, NaN, NaN, '-',  Color='#3df', MarkerSize=8);
h.visionPath.Visible  = 'off';

% target marker
h.targetMark = scatter(ax, NaN, NaN, 150, "o", MarkerEdgeColor='#3df', LineWidth=2.5);

% status LEDs
ledX = -p.L1 + 0.5;
ledY1 = 20 - 0.8;
ledY2 = 20 - 1.8;
h.ledConn   = scatter(ax, ledX, ledY1, 200, 'filled', MarkerFaceColor='#0c0', ...
                      MarkerEdgeColor='#444',LineWidth=3);
h.ledVision = scatter(ax, ledX, ledY2, 200, 'filled', MarkerFaceColor='#555',...
                      MarkerEdgeColor='#444',LineWidth=3);

text(ax, ledX+.7, ledY1, 'Link',   Color='w', FontSize=8, VerticalAlignment='middle', ...
     BackgroundColor='k');
text(ax, ledX+.7, ledY2, 'CV Confirm', Color='w', FontSize=8, VerticalAlignment='middle', ...
     BackgroundColor='k');

%% ===== Control =====
h.toggleBtn = uicontrol(fig, Style='togglebutton', ...
    String='Camera View', ...
    Units='normalized', ...
    Position=[0.85 0.55 0.1 0.08], ...
    Callback=@(src,~) onToggleView(src, fig));
if isempty(cam)
    h.toggleBtn.Enable = 'off';
end

% text box for loop position 1
uicontrol(fig, Style='text', String='Point 1 (x,y):', ...
    Units='normalized', Position=[0.05 0.65 0.1 0.03], ...
    BackgroundColor=fig.Color, ForegroundColor='w', HorizontalAlignment='left');
h.editP1 = uicontrol(fig, Style='edit', String='3, 11', ...
    Units='normalized', Position=[0.05 0.62 0.1 0.04], ...
    BackgroundColor='#1a1300', ForegroundColor='#ffb300', ...
    FontName='Courier', FontSize=10);
% text box for loop position 2
uicontrol(fig, Style='text', String='Point 2 (x,y):', ...
    Units='normalized', Position=[0.05 0.55 0.1 0.03], ...
    BackgroundColor=fig.Color, ForegroundColor='w', HorizontalAlignment='left');
h.editP2 = uicontrol(fig, Style='edit', String='12, 16', ...
    Units='normalized', Position=[0.05 0.52 0.1 0.04], ...
    BackgroundColor='#1a1300', ForegroundColor='#ffb300', ...
    FontName='Courier', FontSize=10);

% run loop button
h.seqBtn = uicontrol(fig, Style='pushbutton', ...
    String='Run Sequence', ...
    Units='normalized', Position=[0.05 0.43 0.1 0.05], ...
    Callback=@(~,~) onRunSequence(fig));

% run trajectory button
h.trajBtn = uicontrol(fig, Style='pushbutton', ...
    String='Run Trajectory', ...
    Units='normalized', Position=[0.05 0.36 0.1 0.05], ...
    Callback=@(~,~) onRunTrajectory(fig));

%% ===== Message Log =====
h.msgBox = uicontrol(fig, Style='listbox', ...
    Units='normalized', Position=[0.0 0.0 1.0 0.1], ...
    String={'[log] Monitor started'}, ...
    FontName='Courier', FontSize=10, ...
    BackgroundColor='#1a1300', ForegroundColor='#ffb300');

%% ===== UserData =====
ud.params          = p;
ud.handles         = h;
ud.camCal          = camCal;
ud.camObj          = cam;
ud.udp             = u;
ud.deviceIP        = UDPdeviceIP;
ud.devicePort      = UDPdevicePort;

ud.ghostTh1        = NaN;
ud.ghostTh2        = NaN;

ud.targetXY        = [NaN NaN];

ud.fkPathXY        = [];
ud.visionPathXY    = [];
ud.visionXY        = [NaN NaN];
ud.visionConfirmed = false;

ud.lastReceiveTime = tic;
ud.cameraMode      = false;
ud.seqRunning      = false;

fig.UserData = ud;
% attached all variables to figure so any callback can 
%modify them by just passing them the figure

%% ===== Timers =====
%watchdog
watchdogTimer               = timer;
watchdogTimer.Period        = 0.5;
watchdogTimer.ExecutionMode = 'fixedRate';
watchdogTimer.TimerFcn      = @(~,~) checkWatchdog(fig);

%vision
visionTimer               = timer;
visionTimer.Period        = 0.06;
visionTimer.ExecutionMode = 'fixedSpacing';
visionTimer.TimerFcn      = @(~,~) runVision(fig);

ud.watchdogTimer = watchdogTimer;
ud.visionTimer   = visionTimer;
fig.UserData     = ud;

start(watchdogTimer);
%% ===== Callback setup =====
fig.WindowButtonDownFcn = @(src, evt) onAxesClick(ax, fig);
fig.CloseRequestFcn     = @(~,~) onFigureClose(fig, watchdogTimer, visionTimer);

flush(u);
u.UserData = fig;
configureCallback(u, "datagram", 1, @onUDPReceive);

%% ===== Main =====
disp("System ready. Click on the workspace to send a target.");
disp("Close the figure to stop.");

while isvalid(fig)
    pause(0.5);
end

disp("Figure closed. Cleaning up.");
configureCallback(u, "off");
delete(u);


%% ===== Callbacks =====

function onAxesClick(ax, fig)
    cp = ax.CurrentPoint;
    x  = cp(1,1);
    y  = cp(1,2);
    sendTarget(fig, x, y);
end

function sendTarget(fig, x, y)
    ud = fig.UserData;
    p  = ud.params;
    [th1, th2, ok] = solveIK(x, y, p.L1, p.L2, p.L3, p.L4, p.L5);
    
    if ~ok
        logMessage(fig, "[warn] Point unreachable");
        return;
    end
    
    ud.ghostTh1 = th1;
    ud.ghostTh2 = th2;
    ud.targetXY = [x, y];
    
    if ~isfield(ud, 'seqRunning') || ~ud.seqRunning
        ud.fkPathXY        = [];
        ud.visionPathXY    = [];
        ud.visionConfirmed = false;
        set(ud.handles.ledVision, MarkerFaceColor='#555');
    end
    
    [Cx,Cy,Dx,Dy,Px,Py,valid] = solveFK(th1, th2, p.L1, p.L2, p.L3, p.L4, p.L5);
    if valid
        h = ud.handles;
        set(h.gL1, XData=[0,Cx],    YData=[0,Cy]);
        set(h.gL2, XData=[p.L5,Dx], YData=[0,Dy]);
        set(h.gL3, XData=[Cx,Px],   YData=[Cy,Py]);
        set(h.gL4, XData=[Dx,Px],   YData=[Dy,Py]);
        set(h.targetMark, XData=x,  YData=y);
        
        if ~isfield(ud, 'seqRunning') || ~ud.seqRunning
            set(h.fkPath,     XData=NaN, YData=NaN);
            set(h.visionPath, XData=NaN, YData=NaN);
        end
    end
    
    fig.UserData = ud;
    msg = sprintf('%.3f,%.3f', x, y);
    write(ud.udp, msg, "string", ud.deviceIP, ud.devicePort);

    if ~isfield(ud, 'seqRunning') || ~ud.seqRunning
        logMessage(fig, sprintf("[sent] x=%.2f  y=%.2f", x, y));
    end
end


function onRunSequence(fig)
    ud = fig.UserData;
    if ud.seqRunning
        logMessage(fig, "[seq] Already running");
        return;
    end

    p1str = strtrim(ud.handles.editP1.String);
    v1    = sscanf(p1str, '%f,%f');
    if numel(v1) ~= 2
        logMessage(fig, "[seq] Invalid Point 1 format — use x,y");
        return;
    end

    p2str = strtrim(ud.handles.editP2.String);
    v2    = sscanf(p2str, '%f,%f');
    if numel(v2) ~= 2
        logMessage(fig, "[seq] Invalid Point 2 format — use x,y");
        return;
    end

    ud.seqRunning    = true;
    ud.handles.seqBtn.Enable = 'off';
    fig.UserData     = ud;

    pauseSec = 1;

    for rep = 1:3
        logMessage(fig, sprintf("[seq] Rep %d/3 — going to P1", rep));
        sendTarget(fig, v1(1), v1(2));
        pauseWithDraw(pauseSec);

        logMessage(fig, sprintf("[seq] Rep %d/3 — going to P2", rep));
        sendTarget(fig, v2(1), v2(2));
        pauseWithDraw(pauseSec);
    end

    logMessage(fig, "[seq] Sequence complete");
    ud = fig.UserData;
    ud.seqRunning = false;
    ud.handles.seqBtn.Enable = 'on';
    fig.UserData  = ud;
end


function onUDPReceive(u, ~)
    fig = u.UserData;
    if ~isvalid(fig), return; end

    d   = read(u, 1, "string");
    raw = strtrim(d.Data);
    if isempty(raw), return; end

    parsed = sscanf(raw, '%f,%f');
    if length(parsed) == 2
        % Stamp receive time for watchdog
        ud                 = fig.UserData;
        ud.lastReceiveTime = tic;
        fig.UserData       = ud;
        updatePlot(fig, parsed(1), parsed(2));
    else
        logMessage(fig, sprintf("[esp] %s", raw));
    end
end


function onToggleView(btn, fig)
    ud = fig.UserData;
    h  = ud.handles;
    if btn.Value
        ud.cameraMode  = true;
        btn.String     = 'Graph View';
        h.camImage.Visible   = 'on';
        h.visionPath.Visible = 'on';
        h.rL1.Visible    = 'off';
        h.rL2.Visible    = 'off';
        h.rL3.Visible    = 'off';
        h.rL4.Visible    = 'off';
        h.fkPath.Visible = 'off';
        fig.UserData = ud;
        start(ud.visionTimer);
        logMessage(fig, "[mode] Camera view active");

    else
        ud.cameraMode  = false;
        btn.String     = 'Camera View';

        h.camImage.Visible   = 'off';
        h.visionPath.Visible = 'off';

        h.rL1.Visible    = 'on';
        h.rL2.Visible    = 'on';
        h.rL3.Visible    = 'on';
        h.rL4.Visible    = 'on';
        h.fkPath.Visible = 'on';

        fig.UserData = ud;
        stop(ud.visionTimer);
        logMessage(fig, "[mode] Graph view active");
    end
end


function onRunTrajectory(fig)
    ud = fig.UserData;
    if ud.seqRunning
        logMessage(fig, "[traj] Sequence or trajectory already running");
        return;
    end
    
    ud.seqRunning = true;
    ud.handles.seqBtn.Enable = 'off';
    ud.handles.trajBtn.Enable = 'off';
    ud.fkPathXY = []; 
    set(ud.handles.fkPath, 'XData', NaN, 'YData', NaN);

    fig.UserData = ud;
    
    % --- Parametric Equations ---
    w=1;

    fx = @(t) 8.5 + 5 * cos(w*t);
    fy = @(t) 12  + 3 * sin(2*w*t);
    
    t_start = 0;
    t_end   = 5*2 * pi/w;
    dt      = 0.05; 
    
    logMessage(fig, "[traj] Generating and streaming trajectory...");
    
    sendTarget(fig, fx(t_start), fy(t_start));
    pauseWithDraw(1.0);
    
    for t = t_start : dt : t_end
        if ~isvalid(fig), return; end 
        x_val = fx(t);
        y_val = fy(t);
        
        sendTarget(fig, x_val, y_val);
        
        pauseWithDraw(dt); 
    end
    
    if isvalid(fig)
        logMessage(fig, "[traj] Trajectory complete");
        ud = fig.UserData;
        ud.seqRunning = false;
        ud.handles.seqBtn.Enable = 'on';
        ud.handles.trajBtn.Enable = 'on';
        fig.UserData = ud;
    end
end


function onFigureClose(fig, watchdogTimer, visionTimer)
    if isvalid(watchdogTimer)
        stop(watchdogTimer);
        delete(watchdogTimer);
    end
    if isvalid(visionTimer)
        stop(visionTimer);
        delete(visionTimer);
    end
    ud = fig.UserData;
    write(ud.udp, "END", "string", ud.deviceIP, ud.devicePort);
    pause(1);
    if ~isempty(ud.camObj)
        delete(ud.camObj);
    end
    delete(fig);
end

%% ===== Plot =====

function updatePlot(fig, c1, c2)
    ud = fig.UserData;
    p  = ud.params;
    h  = ud.handles;
    % Real links
    [Cx,Cy,Dx,Dy,Px,Py,ok] = solveFK(c1, c2, p.L1, p.L2, p.L3, p.L4, p.L5);
    if ok
        set(h.rL1, XData=[0,Cx],    YData=[0,Cy]);
        set(h.rL2, XData=[p.L5,Dx], YData=[0,Dy]);
        set(h.rL3, XData=[Cx,Px],   YData=[Cy,Py]);
        set(h.rL4, XData=[Dx,Px],   YData=[Dy,Py]);

        ud.fkPathXY(end+1,:) = [Px, Py];
        set(h.fkPath, XData=ud.fkPathXY(:,1), YData=ud.fkPathXY(:,2));
    end
    % Ghost links
    if ~isnan(ud.ghostTh1)
        [Cx,Cy,Dx,Dy,Px,Py,ok] = solveFK(ud.ghostTh1, ud.ghostTh2, p.L1, p.L2, p.L3, p.L4, p.L5);
        if ok
            set(h.gL1, XData=[0,Cx],    YData=[0,Cy]);
            set(h.gL2, XData=[p.L5,Dx], YData=[0,Dy]);
            set(h.gL3, XData=[Cx,Px],   YData=[Cy,Py]);
            set(h.gL4, XData=[Dx,Px],   YData=[Dy,Py]);
        end
    end
    fig.UserData = ud;
    drawnow limitrate nocallbacks;
end


function checkWatchdog(fig)
    if ~isvalid(fig), return; end
    ud = fig.UserData;
    h  = ud.handles;
    if toc(ud.lastReceiveTime) > .5
        set(h.ledConn, MarkerFaceColor='#c00');
    else
        set(h.ledConn, MarkerFaceColor='#0c0');
    end
    drawnow limitrate;
end


function pauseWithDraw(sec)
    t0 = tic;
    while toc(t0) < sec
        drawnow limitrate;
        pause(0.05);
    end
end


function runVision(fig)
    if ~isvalid(fig), return; end
    ud  = fig.UserData;
    if isempty(ud.camObj), return; end
    h   = ud.handles;
    cal = ud.camCal;

    frame    = snapshot(ud.camObj);
    hsvFrame = rgb2hsv(frame);

    H    = hsvFrame(:,:,1) * 180;
    S    = hsvFrame(:,:,2);
    V    = hsvFrame(:,:,3);
    mask = (H > 48) & (H < 78) & (S > 0.3) & (V > 0.3);
    set(h.camImage, CData=frame);

    if sum(mask(:)) < 15
        drawnow nocallbacks;
        return;
    end
    % Centroid of blob
    [rows, cols] = find(mask);
    cx_px = mean(cols);
    cy_px = mean(rows);

    % pixel to robot frame
    x_cm =  (cx_px - cal.x0) / cal.px_por_cm;
    y_cm =  (cal.y0 - cy_px) / cal.px_por_cm;

    ud.visionXY = [x_cm, y_cm];

    ud.visionPathXY(end+1,:) = [x_cm, y_cm];
    set(h.visionPath, XData=ud.visionPathXY(:,1), YData=ud.visionPathXY(:,2));

    % ===== Confirmation check =====
    tol = 1.25;
    if ~any(isnan(ud.targetXY))
        dist = norm(ud.visionXY - ud.targetXY);
        if dist < tol
            if ~ud.visionConfirmed
                logMessage(fig, sprintf("[vision] Within tolerance (%.2f cm)", dist));
            end
            ud.visionConfirmed = true;
            set(h.ledVision, MarkerFaceColor='#0c0');
        else
            ud.visionConfirmed = false;
            set(h.ledVision, MarkerFaceColor='#555');
        end
    end

    fig.UserData = ud;
    drawnow limitrate nocallbacks;
end


function logMessage(fig, msg)
    if ~isvalid(fig), return; end
        ud  = fig.UserData;
        h   = ud.handles;
        cur = h.msgBox.String;
    if ischar(cur), cur = {cur}; end
        cur{end+1} = sprintf("[%s]  %s", datestr(now,'HH:MM:SS'), msg);
    if numel(cur) > 4, cur = cur(end-3:end); end
        h.msgBox.String = cur;
        h.msgBox.Value  = length(cur);
end

%% ===== Kinematics =====
function [th1, th2, ok] = solveIK(x, y, L1, L2, L3, L4, L5)
    ok = false; th1 = 0; th2 = 0;

    r1sq  = x^2 + y^2;
    if r1sq > (L1+L3)^2 || r1sq < (L1-L3)^2, return; end
    cosq3 = max(-1, min(1, (r1sq - L1^2 - L3^2) / (2*L1*L3)));
    sinq3 = -sqrt(1 - cosq3^2);
    th1   = atan2d(y, x) - atan2d(L3*sinq3, L1 + L3*cosq3);

    xr    = x - L5;
    r2sq  = xr^2 + y^2;
    if r2sq > (L2+L4)^2 || r2sq < (L2-L4)^2, return; end
    cosq4 = max(-1, min(1, (r2sq - L2^2 - L4^2) / (2*L2*L4)));
    sinq4 = sqrt(1 - cosq4^2);
    th2   = atan2d(y, xr) - atan2d(L4*sinq4, L2 + L4*cosq4);

    ok = true;
end

function [Cx,Cy,Dx,Dy,Px,Py,isValid] = solveFK(th1, th2, L1, L2, L3, L4, L5)
    Cx = L1 * cosd(th1);
    Cy = L1 * sind(th1);
    Dx = L5 + L2 * cosd(th2);
    Dy = L2 * sind(th2);

    d  = sqrt((Dx-Cx)^2 + (Dy-Cy)^2);
    if d > (L3+L4) || d < abs(L3-L4)
        Px=NaN; Py=NaN; isValid=false; return;
    end

    a  = (L3^2 - L4^2 + d^2) / (2*d);
    ht = sqrt(max(0, L3^2 - a^2));

    Px = Cx + (a/d)*(Dx-Cx) - (ht/d)*(Dy-Cy);
    Py = Cy + (a/d)*(Dy-Cy) + (ht/d)*(Dx-Cx);
    isValid = true;
end

function wsPoints = computeWorkspace(p)
    pts = [];
    for th1 = -180:2:180
        for th2 = -180:2:180
            [~,~,~,~,Px,Py,ok] = solveFK(th1, th2, p.L1, p.L2, p.L3, p.L4, p.L5);
            if ok
                pts(end+1,:) = [Px, Py];
            end
        end
    end
    if isempty(pts)
        wsPoints = []; return;
    end
    shp = alphaShape(pts(:,1), pts(:,2), 5);
    [~, wsPoints] = boundaryFacets(shp);
end
