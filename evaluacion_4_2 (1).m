clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 10;            
ts = 0.009;          
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 

x1(1) = 0;              
y1(1) = 0;             
% Calculamos el ángulo inicial exacto basado en el Tramo 1 (dy = x = 0)
dy0 = 0; 
phi(1) = atan2(dy0, 1); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);  
hy = zeros(1,N+1);  
hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); 
w = zeros(1,N); 

for k = 1:N
    tk = t(k);
    dx = 1;
    
    if tk < 3
        % TRAMO 1: f(x) = 0.5*x^2
        dy = tk;
        ddy = 1;
    elseif tk < 7
        % TRAMO 2: f(x) = -(x-5)^2 + 8
        dy = -2*(tk - 5);
        ddy = -2;
    else
        % TRAMO 3: f(x) = 2*exp(x-7)*cos(x-7)
        dy = 2*exp(tk-7)*(cos(tk-7) - sin(tk-7));
        ddy = -4*exp(tk-7)*sin(tk-7);
    end
    
    % Paso 4: Velocidad lineal u(t)
    u(k) = sqrt(dx^2 + dy^2);
    
    % Paso 6: Velocidad angular w(t)
    w(k) = ddy / (1 + dy^2);
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    tk = t(k);
    
    % RECALCULAR ORIENTACIÓN TEÓRICA EXACTA (Paso 5 de la clase)
    if tk < 3
        dy_actual = tk;
    elseif tk < 7
        dy_actual = -2*(tk - 5);
    else
        dy_actual = 2*exp(tk-7)*(cos(tk-7) - sin(tk-7));
    end
    
    phi(k+1) = atan2(dy_actual, 1); 
    
    % Modelo Cinemático Diferencial
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    
    x1(k+1) = x1(k) + xp1*ts; 
    y1(k+1) = y1(k) + yp1*ts; 
    
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene=figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([5 40]);

% Ajuste de límites: x va de 0 a 10, y baja hasta aprox -40 en el Tramo 3
axis([-1 11 -45 10 0 2]); 

scale = 2;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

step = 10; 
for k=1:step:N
    delete(H1);    
    delete(H2);
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    pause(0.01);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;
set(graph,'position',sizeScreen);
subplot(211)
plot(t,u,'b','LineWidth',2),grid on,title('Velocidad Lineal (u)'),ylabel('m/s')
subplot(212)
plot(t,w,'r','LineWidth',2),grid on,title('Velocidad Angular (w)'),ylabel('rad/s')