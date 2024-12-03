% Cenário Fácil
% Configuração inicial
clear; clc; close all;

% Definição dos cenários
scenarios = struct();

% Cenário Fácil
scenarios.easy = struct();
scenarios.easy.gridSize = 50;
scenarios.easy.startPos = [5, 10];
scenarios.easy.goalPos = [45, 45];
scenarios.easy.obstacles = [
    10, 10, 10, 30;
    40, 40, 45, 45;
    18, 35, 20, 40;
    10, 40, 10, 50;
];

% Cenário Intermediário
scenarios.medium = struct();
scenarios.medium.gridSize = 75;
scenarios.medium.startPos = [5, 5];
scenarios.medium.goalPos = [70, 70];
scenarios.medium.obstacles = [
    15, 15, 15, 60;
    35, 10, 35, 65;
    55, 15, 55, 60;
    16, 30, 34, 30;
    36, 45, 54, 45;
    20, 20, 25, 25;
    50, 50, 55, 55;
    40, 5, 43, 10;
];

% Cenário Difícil
scenarios.hard = struct();
scenarios.hard.gridSize = 100;
scenarios.hard.startPos = [5, 5];
scenarios.hard.goalPos = [95, 95];
scenarios.hard.obstacles = [
    20, 20, 20, 80;
    40, 10, 40, 90;
    60, 20, 60, 80;
    80, 10, 80, 90;
    21, 40, 39, 40;
    41, 60, 59, 60;
    61, 40, 79, 40;
    23, 70, 77, 70;
    10, 10, 15, 15;
    85, 85, 90, 90;
    58, 5, 61, 10;
];

% Seleção do cenário (pode ser 'easy', 'medium' ou 'hard')
currentScenario = 'easy';
selected = scenarios.(currentScenario);

% Configuração do ambiente
gridSize = selected.gridSize;
environment = zeros(gridSize);
startPos = selected.startPos;
goalPos = selected.goalPos;
obstacles = selected.obstacles;

% Criação dos obstáculos no ambiente
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

% Plot do ambiente inicial
figure('Name', 'Ambiente de Navegação', 'NumberTitle', 'off');
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
axis equal;
grid on;
legend('show', 'Location', 'best');
title('Ambiente de Navegação com Pontos Inicial e Final');

%% 1. WaveFront Planner
tic;
try
    [pathWaveFront, distWaveFront] = wavefrontPlanner(environment, startPos, goalPos);
catch e
    warning('WaveFront falhou.');
end
timeWaveFront = toc;

%% 2. Algoritmo Genético
tic;
popSize = 50;  % Tamanho da população
maxGen = 100;  % Máximo de gerações
[pathGenetic, distGenetic] = geneticAlgorithmPlanner(environment, startPos, goalPos, popSize, maxGen);
timeGenetic = toc;

%% 3. A* Algorithm
tic;
[pathAStar, distAStar] = aStarPlanner(environment, startPos, goalPos);
timeAStar = toc;

%% Comparação dos métodos
fprintf('\n--- Resultados ---\n');
fprintf('WaveFront: Tempo = %.2f s, Distância = %.2f\n', timeWaveFront, distWaveFront);
fprintf('Algoritmo Genético: Tempo = %.2f s, Distância = %.2f\n', timeGenetic, distGenetic);
fprintf('A*: Tempo = %.2f s, Distância = %.2f\n', timeAStar, distAStar);

figure('Name', 'Comparação de Trajetórias', 'NumberTitle', 'off');
imshow(environment, 'InitialMagnification', 'fit');
hold on;
plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Início');
plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Objetivo');
plot(pathWaveFront(:, 1), pathWaveFront(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', 'WaveFront');
plot(pathGenetic(:, 1), pathGenetic(:, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Genético');
plot(pathAStar(:, 1), pathAStar(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'A*');
legend('Location', 'best');
title('Comparação das Trajetórias Geradas pelos Diferentes Algoritmos');
axis equal;
grid on;
hold off;

%% Funções Auxiliares

% Função WaveFront Planner
function [path, dist] = wavefrontPlanner(env, start, goal)
    [rows, cols] = size(env);
    grid = inf(rows, cols);
    grid(goal(2), goal(1)) = 0;
    
    % Queue para processamento
    queue = [goal(2), goal(1)];
    processed = false(rows, cols);
    
    % 4-conectividade
    directions = [-1, 0; 1, 0; 0, -1; 0, 1];
    
    % Expansão da onda
    while ~isempty(queue)
        current = queue(1, :);
        queue(1, :) = [];
        
        if processed(current(1), current(2))
            continue;
        end
        
        processed(current(1), current(2)) = true;
        
        for d = 1:size(directions, 1)
            ny = current(1) + directions(d, 1);
            nx = current(2) + directions(d, 2);
            
            if ny >= 1 && ny <= rows && nx >= 1 && nx <= cols && ...
               env(ny, nx) == 0 && ~processed(ny, nx)
                
                newDist = grid(current(1), current(2)) + 1;
                if newDist < grid(ny, nx)
                    grid(ny, nx) = newDist;
                    queue = [queue; ny, nx];
                end
            end
        end
    end
    
    % Verificar se um caminho foi encontrado
    if isinf(grid(start(2), start(1)))
        error('Nenhum caminho encontrado pelo WaveFront.');
    end
    
    % Traçar caminho de volta
    path = [start(1), start(2)];
    current = [start(2), start(1)];
    dist = 0;
    
    while ~(current(1) == goal(2) && current(2) == goal(1))
        minVal = inf;
        nextStep = current;
        
        for d = 1:size(directions, 1)
            ny = current(1) + directions(d, 1);
            nx = current(2) + directions(d, 2);
            
            if ny >= 1 && ny <= rows && nx >= 1 && nx <= cols && ...
               env(ny, nx) == 0 && grid(ny, nx) < minVal
                minVal = grid(ny, nx);
                nextStep = [ny, nx];
            end
        end
        
        if isequal(nextStep, current)
            error('Caminho bloqueado no WaveFront.');
        end
        
        % Adicionar ao caminho
        current = nextStep;
        path = [path; current(2), current(1)];
        
        % Calcular distância
        if size(path, 1) > 1
            dist = dist + norm(path(end,:) - path(end-1,:));
        end
    end
end

% Algoritmo Genético
function [path, dist] = geneticAlgorithmPlanner(env, start, goal, popSize, maxGen)
    population = cell(1, popSize);
    fitness = zeros(1, popSize);
    
    for i = 1:popSize
        population{i} = generateSimplePath(start, goal, env);
    end
    
    % Cache de caminhos válidos
    validPathCache = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    
    % Loop principal do algoritmo genético
    for gen = 1:maxGen
        for i = 1:popSize
            fitness(i) = evaluateIndividualFitness(population{i}, env);
        end
        
        % Encontrar o melhor indivíduo
        [currentBestFitness, bestIdx] = max(fitness);
        bestPath = population{bestIdx};
        
        % Seleção
        selected = selectParents(population, fitness);
        
        % Crossover e mutação
        tempPop = cell(1, popSize);
        for i = 1:2:popSize
            if i < popSize
                [child1, child2] = performCrossover(selected{randi(length(selected))}, ...
                                                  selected{randi(length(selected))}, env);
                tempPop{i} = performMutation(child1, env);
                tempPop{i+1} = performMutation(child2, env);
            else
                tempPop{i} = performMutation(selected{randi(length(selected))}, env);
            end
        end
        
        population = tempPop;
    end
    
    path = bestPath;
    dist = sum(sqrt(sum(diff(path).^2, 2)));
end

function fitness = evaluateIndividualFitness(path, env)
    if isValidPath(path, env)
        fitness = -sum(sqrt(sum(diff(path).^2, 2))); % Minimizar distância
    else
        fitness = -inf;
    end
end

% Função auxiliar para realizar cruzamento
function [child1, child2] = performCrossover(parent1, parent2, env)
    maxAttempts = 100;
    child1 = parent1;
    child2 = parent2;
    
    for attempt = 1:maxAttempts
        splitPoint = randi([2, min(size(parent1,1), size(parent2,1))-1]);
        tempChild1 = [parent1(1:splitPoint, :); parent2(splitPoint+1:end, :)];
        tempChild2 = [parent2(1:splitPoint, :); parent1(splitPoint+1:end, :)];
        
        if isValidPath(tempChild1, env) && isValidPath(tempChild2, env)
            child1 = tempChild1;
            child2 = tempChild2;
            break;
        end
    end
end

% Função auxiliar para realizar mutação
function mutated = performMutation(path, env)
    mutated = path;
    maxAttempts = 50;
    
    if rand < 0.1 % Taxa de mutação de 10%
        for i = 2:(size(path,1)-1)
            if rand < 0.1
                for attempt = 1:maxAttempts
                    newPos = path(i, :) + randi([-2, 2], 1, 2);
                    
                    % Verifica se a nova posição é válida
                    if newPos(1) > 0 && newPos(1) <= size(env, 2) && ...
                       newPos(2) > 0 && newPos(2) <= size(env, 1) && ...
                       env(newPos(2), newPos(1)) == 0
                        
                        tempPath = mutated;
                        tempPath(i, :) = newPos;
                        
                        % Verifica se o novo caminho é válido
                        if isValidPath(tempPath, env)
                            mutated = tempPath;
                            break;
                        end
                    end
                end
            end
        end
    end
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

% Função de validação de caminho
function isValid = isValidPath(path, env)
    isValid = true;
    
    % Verifica cada segmento do caminho
    for i = 1:size(path, 1) - 1
        points = bresenham(path(i, 1), path(i, 2), path(i+1, 1), path(i+1, 2));
        
        for j = 1:size(points, 1)
            if points(j,1) < 1 || points(j,1) > size(env,2) || ...
               points(j,2) < 1 || points(j,2) > size(env,1)
                isValid = false;
                return;
            end
            
            if env(points(j,2), points(j,1)) == 1
                isValid = false;
                return;
            end
        end
    end
    margin = 1;
    for i = 1:size(path, 1)
        x = path(i,1);
        y = path(i,2);
        
        for dx = -margin:margin
            for dy = -margin:margin
                newX = x + dx;
                newY = y + dy;
                
                if newX >= 1 && newX <= size(env,2) && ...
                   newY >= 1 && newY <= size(env,1)
                    if env(newY, newX) == 1
                        isValid = false;
                        return;
                    end
                end
            end
        end
    end
end

% Verifica obstáculos por linhas de bresenham
function segment = bresenham(x1, y1, x2, y2)
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

function selected = selectParents(population, fitness)
    [~, idx] = sort(fitness, 'descend');
    selected = population(idx(1:ceil(end/2)));
end

% Função A*
function [path, dist] = aStarPlanner(env, start, goal)
    
    [rows, cols] = size(env);
    closedSet = false(rows, cols);
    
    
    gScore = inf(rows, cols);
    gScore(start(2), start(1)) = 0;
    fScore = inf(rows, cols);
    fScore(start(2), start(1)) = calculateHeuristic(start, goal);
    
    
    cameFrom = zeros(rows, cols, 2);
    directions = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; -1, 1; 1, -1; 1, 1];
    costs = [ones(4,1); sqrt(2)*ones(4,1)];
    
    openSet = [start, fScore(start(2), start(1))];
    
    while ~isempty(openSet)
        % Encontrar nó com menor fScore
        [~, idx] = min(openSet(:,3));
        current = openSet(idx,1:2);
        
        if isequal(current, goal)
            [path, dist] = reconstructPath(cameFrom, current);
            return;
        end
        
        openSet(idx,:) = [];
        closedSet(current(2), current(1)) = true;
        
        % Explorar vizinhos
        for d = 1:size(directions, 1)
            neighbor = current + directions(d,:);
            
            if any(neighbor < 1) || neighbor(1) > cols || neighbor(2) > rows
                continue;
            end
            
            if closedSet(neighbor(2), neighbor(1)) || env(neighbor(2), neighbor(1)) == 1
                continue;
            end
            
            stepCost = costs(d);
            tentative_gScore = gScore(current(2), current(1)) + stepCost;
            
            if tentative_gScore < gScore(neighbor(2), neighbor(1))
                cameFrom(neighbor(2), neighbor(1),:) = current;
                gScore(neighbor(2), neighbor(1)) = tentative_gScore;
                newFScore = tentative_gScore + calculateHeuristic(neighbor, goal);
                fScore(neighbor(2), neighbor(1)) = newFScore;
                
                % Atualizar ou adicionar ao openSet
                existingIdx = find(openSet(:,1) == neighbor(1) & openSet(:,2) == neighbor(2));
                if isempty(existingIdx)
                    openSet = [openSet; neighbor, newFScore];
                else
                    openSet(existingIdx,3) = newFScore;
                end
            end
        end
    end
    
    warning('Nenhum caminho encontrado pelo A*.');
    path = [];
    dist = Inf;
end

function [path, dist] = reconstructPath(cameFrom, current)
    path = current;
    dist = 0;
    
    while true
        prev = squeeze(cameFrom(current(2), current(1), :))';
        if all(prev == 0)
            break;
        end
        dist = dist + norm(current - prev);
        current = prev;
        path = [current; path];
    end
    
    if size(path, 2) ~= 2
        path = reshape(path, [], 2);
    end
end

% Função calculateHeuristic mantida igual
function h = calculateHeuristic(node, goal)
    d = abs(goal - node);
    h = max(d) + (sqrt(2) - 1) * min(d);
end

function neighbors = getNeighbors(node, rows, cols)
    x = node(1);
    y = node(2);
    directions = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; -1, 1; 1, -1; 1, 1];
    candidates = [x, y] + directions;
    valid = all(candidates > 0, 2) & candidates(:,1) <= cols & candidates(:,2) <= rows;
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

% Função de geração de caminho inicial
function path = generateSimplePath(start, goal, env)
    path = [start];
    current = start;
    maxAttempts = 100;
    
    while ~isequal(current, goal)
        direction = sign(goal - current);
        bestDist = inf;
        bestNext = current;
        
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    continue;
                end
                
                next = current + [dx, dy];
        
                if next(1) > 0 && next(1) <= size(env, 2) && ...
                   next(2) > 0 && next(2) <= size(env, 1) && ...
                   env(next(2), next(1)) == 0
                    
                    dist = norm(goal - next);
                    if dist < bestDist
                        bestDist = dist;
                        bestNext = next;
                    end
                end
            end
        end
        
        if isequal(bestNext, current)
            for attempt = 1:maxAttempts
                randDir = randi([-1, 1], 1, 2);
                next = current + randDir;
                
                if next(1) > 0 && next(1) <= size(env, 2) && ...
                   next(2) > 0 && next(2) <= size(env, 1) && ...
                   env(next(2), next(1)) == 0
                    bestNext = next;
                    break;
                end
            end
        end
        
        current = bestNext;
        path = [path; current];
        
        if size(path, 1) > 1000
            break;
        end
    end
end
