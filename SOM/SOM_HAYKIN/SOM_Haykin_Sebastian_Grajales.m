    %Ejemplo de un mapa conceptual hecho por un SOM del libro de 
%Simon Haykin - Neural Networks and Learning Machines (Cap 9 - Sección 4)
%author: Pupilos en formación - Sebastián Grajales/Daniel Sánchez

clc;clear;close all

nEpocas = input("Número de épocas(2000): ");    %Número de épocas
eta_i = input("Eta(0.4): "); %Constante de aprendizaje
radio_i = input("Radio inicial(4): "); %Radio Inicial
radio_f = input("Radio final(1): "); %Radio Final
time = input("Tiempo de espera entre graficas(pause): ")
fps = input("Cada cuantas épocas mostrar la grafica(200): ")                                   
dataset = csvread("Caracteristicas.csv",1,1);   %Obtención de las caracteristicas
x_s = eye(16)*0.2;                              %Creación de la matriz con los identificadores
x = cat(1,x_s,dataset);                         %Concatenación

%Normalización de los Vectores
for i = 1:16                                    
    x(:,i) = x(:,i)/(norm(x(:,i)));
end

%Creación de la parrilla y inicialización de los pesos
map = cell(10,10);                               
for i = 1:numel(map)
weights =  -0.5 + (1)*rand(29,1);  %Vector de pesos de cada neurona
map{i} = weights/norm(weights);%Normalización de los pesos
end

%Parte de la creación de las Graficas
animals = ["Paloma", "Gallina", "Pato", "Ganso", "Búho", "Halcón", "Águila","Zorro", "Perro", "Lobo", "Gato", "Tigre", "León","Caballo", "Cebra", "Vaca"];
cte = 1; %variable de impresión
cte_g= 1; %variable grafica

for t = 1:nEpocas %Cicilo de para las epocas
    
    eta = eta_i*((0.01/eta_i)^(t/nEpocas));  %η que varía con las épocas
    radio = round(radio_i +(radio_f-radio_i)*(t/nEpocas)); %Radio que varia con las epocas
    x_a = x(:,randperm(16)); %Variación del orden de los datos de muestra
    
    
    for p = 1:16 %Ciclo para mostrar todas las muestras
        aux = zeros(1,numel(map)); %Arreglo aux
        
        %Ciclo para buscar la neurona ganadora la muestra
   for w = 1:numel(map)
    e = x_a(:,p)' * map{w} / norm(map{w}); % Calcular producto escalar
    aux(w) = e; % Guardar en vector auxiliar
    end
        g = find(aux==max(aux),1);gana=map{g};  %Índice de la neurona más parecida a patrón
%Creación de la vecindad
d = ceil(g/10); % Obtención de la fila a partir del índice
u = (mod(g,10)*(mod(g,10)~=0)+(10)*(mod(g,10)==0)); % Obtención de la columna a partir del índice
for j = 1:numel(map)
    [jj,ii] = ind2sub([10 10], j); % Obtención de fila y columna de la neurona actual
    if abs(ii-d)<=radio && abs(jj-u)<=radio % Selección de la vecindad
        vecino = map{j}; % Neurona a actualizar
        map{j} = vecino + eta*(x_a(:,p)-vecino); % Actualización de los pesos
        map{j} = map{j}/norm(map{j}); % Normalización de los pesos
    end
end
        %end
    end
if cte_g == fps
clf;
gx=0; aux = zeros(1,16); u=1; d=1;
%Primera Grafica - Vecindades
for j= 1:numel(map)
    aux = zeros(1,16);
   for p= 1:16
        neurona = map{j};      
        e = norm(x(:,p)-neurona); %Producto escalar
        aux(p) = e;   %Auxiliar guarda productos escalares
    end
    g = find(aux==min(aux),1); 
    ax = gca;
    figure(1);
    title(['Reino Animal - Epocas: ' num2str(t)],'FontSize', 16);
    ax = gca;
    xlim([0 10]);
    ylim([0 10]);
    if g<8
    rectangle('Position',[u-1,d-1,1,1],'FaceColor',"#e4f1cb") 
    elseif g>=8 && g<=13
    rectangle('Position',[u-1,d-1,1,1],'FaceColor',"#bde0ff")
    else
        rectangle('Position',[u-1,d-1,1,1],'FaceColor',"#dbbdff")
    end
    text(u-0.7, d-0.5, animals{g},'FontSize', 14) %Grafica el animal con el mayor estimulo a la neurona
    grid on
    hold on
    u = u+1;
    if u == 11
        u=1; d=d+1;
    end

end
cte_g = 0;
pause(time)
end
    
    if cte == 200
     disp(t)
     cte=0;
    end
    cte = cte+1;
    cte_g= cte_g+1;
end
aux = zeros(1,16);
%Segunda Grafica - Neuronas Ganadoras
for p= 1:16
    for j= 1:numel(map) 
        neurona = map{j};      
        e = norm(x(:,p)-neurona); %Producto escalar
        aux(j) = e;                         %Auxiliar guarda productos escalares
    end
    g = find(aux==min(aux),1); 
    d_1 = ceil(g/10); u_1 = (mod(g,10)*(mod(g,10)~=0)+(10)*(mod(g,10)==0));
    figure(2);
    title('Animales');
    ax = gca;
    xlim([0 10]);
    ylim([0 10]);
    text(u_1-0.7, d_1-0.5, animals{p})
    grid on
    hold on
end
