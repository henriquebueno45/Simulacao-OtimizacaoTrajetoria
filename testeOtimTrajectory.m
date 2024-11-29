% Configuração inicial
clear; clc; close all;

% Dimensões do ambiente
gridSize = 50;
environment = zeros(gridSize);

% Posição inicial e objetivo
startPos = [5, 10];
goalPos = [45, 45];

% Definição de obstáculos
obstacles = [
    10, 10, 10, 30; % [x1, y1, x2, y2]
    %25, 15, 35, 25;
    40, 40, 45, 45;
    18, 35, 20, 40;
];
for i = 1:size(obstacles, 1)
    environment(obstacles(i, 2):obstacles(i, 4), obstacles(i, 1):obstacles(i, 3)) = 1;
end

% Verificar se o ponto inicial e o ponto final estão em um obstáculo
if environment(startPos(2), startPos(1)) == 1
    warning('Ponto inicial está em um obstáculo. Encontrando nova posição livre.');
    startPos = findFreeCell(environment, startPos);
end

if environment(goalPos(2), goalPos(1)) == 1
    warning('Ponto final está em um obstáculo. Encontrando nova posição livre.');
    goalPos = findFreeCell(environment, goalPos);
end

% Plot do ambiente
figure;
imagesc(environment);
colormap(gray);
hold on;

% Plotar ponto inicial
plot(startPos(1), startPos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'green', 'DisplayName', 'Start');

% Plotar ponto final
plot(goalPos(1), goalPos(2), 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', ...
    'MarkerFaceColor', 'red', 'DisplayName', 'Goal');

hold off;
legend('show');

%% 1. WaveFront Planner
tic;
[pathWaveFront, distWaveFront] = wavefrontPlanner(environment, startPos, goalPos);
timeWaveFront = toc;

%% 2. Algoritmo Genético
tic;
[pathGenetic, distGenetic] = geneticAlgorithmPlanner(environment, startPos, goalPos, 50, 100);
timeGenetic = toc;

%% 3. A* Algorithm
tic;
[pathAStar, distAStar] = aStarPlanner(environment, startPos, goalPos);

if ~isempty(pathAStar)
    plot(pathAStar(:, 1), pathAStar(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'A*');
else
    warning('Nenhum caminho encontrado pelo A*.');
end

timeAStar = toc;

%% Comparação dos métodos
fprintf('\n--- Resultados ---\n');
fprintf('WaveFront: Tempo = %.2f s, Distância = %.2f\n', timeWaveFront, distWaveFront);
fprintf('Algoritmo Genético: Tempo = %.2f s, Distância = %.2f\n', timeGenetic, distGenetic);
fprintf('A*: Tempo = %.2f s, Distância = %.2f\n', timeAStar, distAStar);

% Plot das trajetórias
figure;
imshow(environment, 'InitialMagnification', 'fit');
hold on;
plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(pathWaveFront(:, 1), pathWaveFront(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', 'WaveFront');
plot(pathGenetic(:, 1), pathGenetic(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Genético');
plot(pathAStar(:, 1), pathAStar(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'A*');
legend;
title('Comparação de trajetórias');
hold off;

%% Funções Auxiliares

% Função WaveFront Planner
function [path, dist] = wavefrontPlanner(env, start, goal)
    [rows, cols] = size(env);
    grid = inf(rows, cols);
    grid(goal(2), goal(1)) = 0;
    
    queue = goal;
    directions = [-1, 0; 1, 0; 0, -1; 0, 1]; % Up, down, left, right
    
    while ~isempty(queue)
        [current, queue] = deal(queue(1, :), queue(2:end, :));
        
        for d = 1:size(directions, 1)
            neighbor = current + directions(d, :);
            if neighbor(1) > 0 && neighbor(1) <= cols && neighbor(2) > 0 && neighbor(2) <= rows
                if env(neighbor(2), neighbor(1)) == 0 && grid(neighbor(2), neighbor(1)) > grid(current(2), current(1)) + 1
                    grid(neighbor(2), neighbor(1)) = grid(current(2), current(1)) + 1;
                    queue = [queue; neighbor];
                end
            end
        end
    end
    
    % Traçando o caminho de volta
    current = start;
    path = current;
    dist = 0;
    
    while ~isequal(current, goal)
        neighbors = getNeighbors(current, rows, cols);
        validNeighbors = neighbors((arrayfun(@(j) grid(neighbors(j, 2), neighbors(j, 1)), 1:size(neighbors, 1))) < inf, :);
        
        if isempty(validNeighbors)
            error('Nenhum caminho encontrado pelo WaveFront.');
        end
        
        [~, idx] = min(arrayfun(@(k) grid(validNeighbors(k, 2), validNeighbors(k, 1)), 1:size(validNeighbors, 1)));
        current = validNeighbors(idx, :);
        path = [path; current];
        dist = dist + 1;
    end
end

% Função Algoritmo Genético
function [path, dist] = geneticAlgorithmPlanner(env, start, goal, popSize, maxGen)
    population = generateInitialPopulation(popSize, start, goal, env);
    for gen = 1:maxGen
        fitness = evaluateFitness(population, env);
        selected = selectParents(population, fitness);
        offspring = crossover(selected, popSize, env);
        population = mutate(offspring, env);
    end
    [~, bestIdx] = max(evaluateFitness(population, env));
    path = population{bestIdx};
    dist = sum(sqrt(sum(diff(path).^2, 2)));
end

function population = generateInitialPopulation(popSize, start, goal, env)
    maxAttempts = 100;
    population = cell(1, popSize);
    for i = 1:popSize
        for attempt = 1:maxAttempts
            path = incrementalPathGeneration(start, goal, env);
            if isValidPath(path, env)
                population{i} = path;
                break;
            end
        end
        if isempty(population{i})
            error('Não foi possível gerar um caminho inicial válido dentro do limite de tentativas.');
        end
    end
end

function path = incrementalPathGeneration(start, goal, env)
    path = start;
    current = start;
    while ~isequal(current, goal)
        direction = sign(goal - current);
        nextStep = current + direction;
        if all(nextStep > 0) && all(nextStep <= size(env)) && env(nextStep(2), nextStep(1)) == 0
            current = nextStep;
        else
            % Tente mover lateralmente se bloqueado
            if direction(1) ~= 0 && env(current(2), current(1) + direction(1)) == 0
                current(1) = current(1) + direction(1);
            elseif direction(2) ~= 0 && env(current(2) + direction(2), current(1)) == 0
                current(2) = current(2) + direction(2);
            else
                % Se completamente bloqueado, tenta um movimento alternativo
                alternativeMoves = [1, 0; 0, 1; -1, 0; 0, -1];
                for move = alternativeMoves'
                    tentative = current + move';
                    if all(tentative > 0) && all(tentative <= size(env)) && env(tentative(2), tentative(1)) == 0
                        current = tentative;
                        break;
                    end
                end
            end
        end
        path = [path; current];
    end
end

function isValid = isValidPath(path, env)
    isValid = true;
    for i = 1:size(path, 1) - 1
        segment = bresenham(path(i, 1), path(i, 2), path(i+1, 1), path(i+1, 2));
        for j = 1:size(segment, 1)
            if env(segment(j, 2), segment(j, 1)) == 1
                isValid = false;
                return;
            end
        end
    end
end

function segment = bresenham(x1, y1, x2, y2)
    % Bresenham's Line Algorithm to get the line between two points
    line = [];
    steep = abs(y2 - y1) > abs(x2 - x1);
    if steep
        [x1, y1] = deal(y1, x1);
        [x2, y2] = deal(y2, x2);
    end
    if x1 > x2
        [x1, x2] = deal(x2, x1);
        [y1, y2] = deal(y2, y1);
    end
    deltax = x2 - x1;
    deltay = abs(y2 - y1);
    error = 0;
    y = y1;
    if y1 < y2
        ystep = 1;
    else
        ystep = -1;
    end
    for x = x1:x2
        if steep
            line = [line; y, x];
        else
            line = [line; x, y];
        end
        error = error + deltay;
        if 2 * error >= deltax
            y = y + ystep;
            error = error - deltax;
        end
    end
    segment = line;
end

function fitness = evaluateFitness(population, env)
    fitness = zeros(1, length(population));
    for i = 1:length(population)
        path = population{i};
        if isValidPath(path, env)
            fitness(i) = -sum(sqrt(sum(diff(path).^2, 2))); % Minimizar distância
        else
            fitness(i) = -inf;
        end
    end
end

function selected = selectParents(population, fitness)
    [~, idx] = sort(fitness, 'descend');
    selected = population(idx(1:ceil(end/2)));
end

function offspring = crossover(selected, popSize, env)
    maxAttempts = 100;
    offspring = cell(1, popSize);
    for i = 1:popSize
        for attempt = 1:maxAttempts
            p1 = selected{randi(length(selected))};
            p2 = selected{randi(length(selected))};
            splitPoint = randi([2, min(size(p1,1), size(p2,1))-1]);
            child = [p1(1:splitPoint, :); p2(splitPoint+1:end, :)];
            if isValidPath(child, env)
                offspring{i} = child;
                break;
            end
        end
        if isempty(offspring{i})
            warning('Não foi possível gerar um descendente válido dentro do limite de tentativas.');
            offspring{i} = selected{randi(length(selected))};
        end
    end
end

function mutated = mutate(offspring, env)
    for i = 1:length(offspring)
        if rand < 0.1
            idx = randi(size(offspring{i}, 1));
            newPos = offspring{i}(idx, :) + randi([-1, 1], 1, 2);
            if newPos(1) > 0 && newPos(1) <= size(env, 2) && newPos(2) > 0 && newPos(2) <= size(env, 1)
                if env(newPos(2), newPos(1)) == 0
                    offspring{i}(idx, :) = newPos;
                end
            end
        end
    end
    mutated = offspring;
end

% Função A* Planner
function [path, dist] = aStarPlanner(env, start, goal)
    [rows, cols] = size(env);
    openSet = start;
    cameFrom = zeros(rows, cols, 2);
    gScore = inf(rows, cols);
    gScore(start(2), start(1)) = 0;
    fScore = inf(rows, cols);
    fScore(start(2), start(1)) = heuristic(start, goal);
    
    while ~isempty(openSet)
        [~, idx] = min(arrayfun(@(i) fScore(openSet(i, 2), openSet(i, 1)), 1:size(openSet, 1)));
        current = openSet(idx, :);
        
        if isequal(current, goal)
            [path, dist] = reconstructPath(cameFrom, current, gScore(goal(2), goal(1)));
            return;
        end
        
        openSet(idx, :) = [];
        neighbors = getNeighbors(current, rows, cols);
        
        for n = neighbors'
            if env(n(2), n(1)) == 1
                continue;
            end
            tentative_gScore = gScore(current(2), current(1)) + 1;
            if tentative_gScore < gScore(n(2), n(1))
                cameFrom(n(2), n(1), :) = current;
                gScore(n(2), n(1)) = tentative_gScore;
                fScore(n(2), n(1)) = gScore(n(2), n(1)) + heuristic(n', goal);
                if ~ismember(n', openSet, 'rows')
                    openSet = [openSet; n'];
                end
            end
        end
    end
    
    warning('Nenhum caminho encontrado pelo A*.');
    path = [];
    dist = Inf;
end

function [path, dist] = reconstructPath(cameFrom, current, dist)
    path = current;
    while any(cameFrom(current(2), current(1), :))
        current = squeeze(cameFrom(current(2), current(1), :))';
        path = [current; path];
    end
end

function h = heuristic(node, goal)
    h = norm(goal - node);
end

function neighbors = getNeighbors(node, rows, cols)
    x = node(1);
    y = node(2);
    candidates = [x-1, y; x+1, y; x, y-1; x, y+1];
    valid = candidates(:, 1) > 0 & candidates(:, 1) <= cols & ...
            candidates(:, 2) > 0 & candidates(:, 2) <= rows;
    neighbors = candidates(valid, :);
end

function freeCell = findFreeCell(environment, position)
    [rows, cols] = size(environment);
    x = position(1);
    y = position(2);

    if environment(y, x) == 0
        freeCell = position;
        return;
    end

    for radius = 1:max(rows, cols)
        [X, Y] = meshgrid(x-radius:x+radius, y-radius:y+radius);
        candidates = [X(:), Y(:)];
        candidates = candidates(all(candidates > 0 & candidates <= [cols, rows], 2), :);

        for i = 1:size(candidates, 1)
            cx = candidates(i, 1);
            cy = candidates(i, 2);
            if environment(cy, cx) == 0
                freeCell = [cx, cy];
                return;
            end
        end
    end

    freeCell = [];
end
