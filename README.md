# UGV-Steering-based-Unmanned-Ground-Vehicle-
UGV(Steering-based Unmanned Ground Vehicle)


%% 5. Ackermann-style UGV Simulation
% Kinematic Bicycle Model + Pure Pursuit

figure('Color','w');

imagesc([1 W], [1 H], map);
set(gca,'YDir','normal');

hold on;
axis equal;
xlim([0 W]);
ylim([0 H]);
grid on;

xlabel('X Position');
ylabel('Y Position');

title('Steering-based Hospital UGV Simulation');

colormap([1 1 1; 0.2 0.2 0.2]);

% Start and Goal
plot(start(1), start(2), 'go', ...
    'MarkerSize', 10, 'MarkerFaceColor','g');

plot(goal(1), goal(2), 'rp', ...
    'MarkerSize', 14, 'MarkerFaceColor','r');

% A* planned path
plot(path(:,1), path(:,2), ...
    'b--', 'LineWidth', 1.5);

% UGV parameters
L = 2.8;               % Wheelbase
vehicleLength = 4.0;
vehicleWidth = 2.4;

v = 2.0;               % Forward velocity
dt = 0.05;             % Time step

maxSteering = deg2rad(30);
lookAhead = 2.0;

% Initial robot position
x = start(1);
y = start(2);

theta = atan2(path(2,2)-path(1,2), ...
              path(2,1)-path(1,1));

delta = 0;

% Vehicle body
bodyLocal = [
    -vehicleLength/2, -vehicleWidth/2;
     vehicleLength/2, -vehicleWidth/2;
     vehicleLength/2,  vehicleWidth/2;
    -vehicleLength/2,  vehicleWidth/2
];

bodyPatch = patch(nan,nan,[0.2 0.5 0.9], ...
    'EdgeColor','k','LineWidth',1.5);

% Heading indicator
headingLine = plot(nan,nan,'r-', ...
    'LineWidth',2);

% Steering indicator
steeringLine = plot(nan,nan,'m-', ...
    'LineWidth',3);

% Actual trajectory
trail = animatedline('Color','c', ...
    'LineWidth',2);

% Simulation parameters
lastIndex = 1;
maxIterations = 20000;
actualDistance = 0;
maxSteeringUsed = 0;

robotHistory = [x y];

tic;

%% 6. Pure Pursuit Control Loop

for iteration = 1:maxIterations

    % Distance to goal
    goalDistance = hypot(goal(1)-x, goal(2)-y);

    if goalDistance < 1.0
        break;
    end

    % Find nearest point on remaining path
    searchEnd = min(size(path,1),lastIndex+30);

    segment = path(lastIndex:searchEnd,:);

    distances = sum((segment-[x y]).^2,2);

    [~,nearest] = min(distances);

    nearestIndex = lastIndex + nearest - 1;
    lastIndex = nearestIndex;

    % Find look-ahead target
    remainingPath = path(nearestIndex:end,:);

    cumulativeDistance = [0; ...
        cumsum(vecnorm(diff(remainingPath),2,2))];

    targetIndex = find( ...
        cumulativeDistance >= lookAhead,1);

    if isempty(targetIndex)
        targetIndex = size(remainingPath,1);
    end

    target = remainingPath(targetIndex,:);

    % Calculate heading error
    alpha = atan2(target(2)-y,target(1)-x)-theta;

    alpha = atan2(sin(alpha),cos(alpha));

    % Pure Pursuit steering control
    Ld = max(hypot(target(1)-x,target(2)-y),0.1);

    delta = atan2(2*L*sin(alpha),Ld);

    % Steering angle limitation
    delta = max(-maxSteering, ...
        min(maxSteering,delta));

    maxSteeringUsed = max(maxSteeringUsed,abs(delta));

    % Kinematic Bicycle Model
    xNext = x + v*cos(theta)*dt;

    yNext = y + v*sin(theta)*dt;

    thetaNext = theta + ...
        (v/L)*tan(delta)*dt;

    thetaNext = atan2(sin(thetaNext), ...
                      cos(thetaNext));

    % Collision check at robot reference point
    ix = round(xNext);
    iy = round(yNext);

    if ix < 1 || ix > W || iy < 1 || iy > H
        warning('Robot reached map boundary.');
        break;
    end

    if map(iy,ix) == 1
        warning('Collision detected. Simulation stopped.');
        break;
    end

    % Update position
    actualDistance = actualDistance + ...
        hypot(xNext-x,yNext-y);

    x = xNext;
    y = yNext;
    theta = thetaNext;

    robotHistory(end+1,:) = [x y];

    % Rotate vehicle body
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    worldCorners = (R*bodyLocal')' + [x y];

    set(bodyPatch, ...
        'XData',worldCorners(:,1), ...
        'YData',worldCorners(:,2));

    % Heading direction
    set(headingLine, ...
        'XData',[x, x+3*cos(theta)], ...
        'YData',[y, y+3*sin(theta)]);

    % Front steering direction
    frontX = x + (L/2)*cos(theta);
    frontY = y + (L/2)*sin(theta);

    set(steeringLine, ...
        'XData',[frontX, frontX+1.5*cos(theta+delta)], ...
        'YData',[frontY, frontY+1.5*sin(theta+delta)]);

    % Draw actual trajectory
    addpoints(trail,x,y);

    drawnow limitrate;
    pause(0.01);

end

simulationTime = toc;

%% 7. Performance Evaluation

finalDistance = hypot(goal(1)-x,goal(2)-y);

fprintf('\n--- UGV Simulation Results ---\n');

fprintf('Final position: X=%.2f, Y=%.2f\n',x,y);

fprintf('Remaining goal distance: %.2f\n',finalDistance);

fprintf('Actual travel distance: %.2f\n',actualDistance);

fprintf('Maximum steering angle: %.2f degrees\n', ...
    rad2deg(maxSteeringUsed));

fprintf('Simulation time: %.2f seconds\n',simulationTime);

fprintf('Number of iterations: %d\n',iteration);

if finalDistance < 1.0
    fprintf('Status: GOAL REACHED\n');
else
    fprintf('Status: GOAL NOT REACHED\n');
end

title('Hospital UGV - Steering Control Simulation');

legend('Start','Goal','A* Path', ...
    'UGV Body','Heading','Steering Direction','Trajectory');

hold off;
