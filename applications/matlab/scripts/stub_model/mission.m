%   GENERATE MISSION TABLE
close all
PLOT = 0;
DEBUG = 0; % print debug for turns and gamma
HAVE_LATLON = 0; % 1=lat lon waypoint source; 0=heading, distance waypoint

D2R = pi/180;
R2D = 180/pi;
FT2M = 0.3048;

DEG2FT = 60*6076;
DEG2M = DEG2FT*FT2M;

M2DEG = 1/DEG2M;
FT2DEG = 1/DEG2FT;

KTS2MPS = 1/1.94384;

if HAVE_LATLON
    % ***** MISSION PART 1a: LAT/LON WAYPOINTS *****
    % USE 1b IF DOING DIST/HEADING INSTEAD

    % SF-Candlestick
    lonlat = kml2lla('SF-Candlestick.kml');
    altsFT = lonlat(3,:);
    speedKTS = [0 0.5 35 35 90   90 70 70 20 4   0];

    % START OF MISSION STRUCTURE
    uam(1,:) = lonlat(2,:); % lat deg
    uam(2,:) = lonlat(1,:); % lon deg
   
    % FOR SIMULINK ONLY
    baselat = lonlat(2,1);
    baselon = lonlat(1,1);
    basealt = 0*FT2M;
    basehdg = 159*D2R;

    % USED FOR PLOT ONLY 
    midlat = (lonlat(2,1)+ lonlat(2,end))/2;

    % CALC HEADING AND DISTANCE BETWEEN WAYPOINTS
    hdgsD = [0]; % HEADING TO THIS INDEX
    dist1_FT = [0]; % SEGMENT DISTANCE TO THIS INDEX
    distsFT = [0]; % CUMULATIVE DISTANCE TO THIS INDEX
    for i=2:size(lonlat,2)
        % SPHERICAL DISTANCE
        [d, az] = distance( lonlat(2,i-1), lonlat(1,i-1), lonlat(2,i), lonlat(1,i) );
        distsFT(i) = distsFT(i-1) + d*DEG2FT;
        dist1_FT(i) = d*DEG2FT;
        hdgsD(i) = az;
    end

    % CLEAN UP
    clear d az

%if HAVE_LATLON
else
    % ***** MISSION PART 1b: BASE DISTANCES TO SEED WAYPOINTS *****
    % USE 1a IF YOU HAVE THE LON/LAT DIRECTLY
    
    distsFT  = [0 0.5*6076/3600*6*4 414 1359 16717  13661 5280*3.5 16450 2364 2854  1903 20*6076/3600*2*4 0  ];
    altsFT   = [0                50 85  200  750    1500  1500     750   550  250   50   50               0  ];
    hdgsD    = [0                 0 0   0    0      0     -90      -90   -90  -90   -90  -90              -90];
    speedKTS = [0               0.5 10  35   90     90    130      130   70   70    20   4                0  ];
 
    % FOR SIMULINK ONLY
    baselat = 32.80045;
    baselon = -117.14736;
    basealt = 0;
    basehdg = 0;

    % USED TO SCALE LON FROM EAST DISTANCE
    midlat = baselat;
    
    % EMPTY MISSION DATA STRUCTURE
    uam = zeros(7, size(distsFT,2));
    
    % COMPRESS DISTANCES
    distsFT = distsFT/1; % COMPRESS THE DISTANCE
    dist1_FT = distsFT; % SAVE A COPY AS SEGMENT DISTANCE
    distN_FT = zeros(size(distsFT,1));
    distE_FT = zeros(size(distsFT,1));
    
    % ACCUMULATE DISTANCES ABOVE, PER PRESCRIBED HEADINGS
    hdgsR = hdgsD*D2R;
    for i=1:size(distsFT,2)
        if i>1
            distsFT(i) = distsFT(i-1) + dist1_FT(i);
        end
        if i==1
            distN_FT(i) = dist1_FT(i)*cos(hdgsR(i));
            distE_FT(i) = dist1_FT(i)*sin(hdgsR(i));
        else
            distN_FT(i) = distN_FT(i-1) + dist1_FT(i)*cos(hdgsR(i));
            distE_FT(i) = distE_FT(i-1) + dist1_FT(i)*sin(hdgsR(i));
        end
    end
    
    % CLEAN UP PART 1
    clear hdgsR
end

% ***** MISSION PART 2, USE PART 1 AND COMPLETE THE REST OF THE MISSION

% CALCULATE HEADING CHANGE COMING UP, AND EARLY CAPTURE DISTANCE
turnD = [hdgsD(2:end) hdgsD(end)] - hdgsD;
for i=1:size(turnD,2)
    if turnD(i) < -180
        turnD(i) = turnD(i) + 360;
    elseif turnD(i) > 180
        turnD(i) = turnD(i) - 360;
    end
end
% assume 30 deg/sec, times speed to get capture distance
capFT = abs(turnD) ./ 30 .* speedKTS.*6076./3600;

% PRINT WP TURNS FOR DEBUGGING
if DEBUG
    disp('waypoint heading(deg), turn(deg), speed(kts), capture distance(ft):') 
    str = '';
    htvc = [hdgsD; turnD; speedKTS; capFT];
    for i=1:size(htvc,1)
        for j=1:size(htvc,2)
            str = [str sprintf('%.1f ',htvc(i,j))];
        end
        str = [str sprintf(';\n')];
    end
    disp(str)
end

% CALCULATE GAMMAS TO CURRENT POINT
rise = altsFT-[0 altsFT(1:end-1)];
gammaD = atan2(rise,dist1_FT)*180/pi;

% PRINT WP GAMMAS FOR DEBUGGING
if DEBUG
    disp('waypoint distance(ft), rise(ft), gamma(deg):')
    str = '';
    drg = [dist1_FT; rise; gammaD];
    for i=1:size(drg,1)
        for j=1:size(drg,2)
            str = [str sprintf('%.1f ',drg(i,j))];
        end
        str = [str sprintf(';\n')];
    end
    disp(str)
end

% CLEAN UP PART 2
clear dist1_FT turnD htvc rise drg

%***** MISSION PART 3: FILL THE STRUCTURE

% COPY DISTANCES TO LAT LON ROWS
if ~HAVE_LATLON
    uam(1,:) = baselat + distN_FT*FT2DEG; % lat deg
    uam(2,:) = baselon + distE_FT*FT2DEG/cos(midlat*D2R); % lon deg
end

uam(3,:) = altsFT*FT2M; % alt m
uam(4,:) = hdgsD; % heading deg
uam(5,:) = gammaD; % gamma deg
uam(6,:) = speedKTS*KTS2MPS; % speed kts to m/s
uam(7,:) = -capFT*FT2M; % late+/early- WP capture m

% PLOT FOR DEBUGGING
if PLOT
    figure;
    plot(distsFT/6076,altsFT,'-x');
    for i=1:size(uam,2)
        text(distsFT(i)/6076,altsFT(i),sprintf('%d\n%.1fkts,%.1fdeg',i,speedKTS(i),gammaD(i)));
    end
    title('Waypoint Altitude Profile')
    xlabel('distance (nautical mile)');
    ylabel('altitude(feet)');
end
if PLOT
    % PLOT LAT LON ALT
    figure;
    plot3(uam(2,:), uam(1,:), altsFT);
    for i=1:size(uam,2)
        text(uam(2,i),uam(1,i),altsFT(i),num2str(i));
    end
    title('Waypoint 3D plot')
    xlabel('Longitude (deg)')
    ylabel('Latitude (deg)')
end
if PLOT
    % PLOT N, E, UP
    figure;
    plot3( (uam(2,:)-baselon)*DEG2FT*cos(midlat*D2R), (uam(1,:)-baselat)*DEG2FT, altsFT);
    for i=1:size(uam,2)
        text( (uam(2,i)-baselon)*DEG2FT*cos(midlat*D2R), (uam(1,i)-baselat)*DEG2FT, altsFT(i),num2str(i));
    end
    title('Waypoint 3D plot')
    xlabel('East (meters)')
    ylabel('North (meters)')
end

% CLEAN UP PART 3
clear lonlat midlat distsFT altsFT hdgsD gammaD speedKTS capFT
clear D2R R2D FT2M DEG2FT DEG2M M2DEG FT2DEG KTS2MPS

% OPTIONAL LABEL FOR DEBUG VIEWING
lab = ['lat d';'lon d';'alt m';'hdg d';'gam d';'spd m'; 'cap m'];

% PRINT TABLE, FULL DECIMAL FOR LAT LON ONLY
if DEBUG
    str = '[';
    for j=1:size(uam,1)
        for i=1:size(uam,2)
            if j <= 2
                str = [str sprintf('%.8f ',uam(j,i))];
            else
                str = [str sprintf('%.3f ',uam(j,i))];
            end
        end
        str = [str sprintf('; %% %s\n', lab(j,:))];
    end
    str = [str sprintf(']')];
    disp('WP lookup table:')
    disp(str)
end

% CLEAN UP PART 3
clear str i j lab
clear DEBUG PLOT HAVE_LATLON
