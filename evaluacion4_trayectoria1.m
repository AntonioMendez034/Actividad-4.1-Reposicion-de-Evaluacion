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
% Calculamos el ángulo inicial exacto basado en la derivada en t=0
dy0 = 2*exp(-0.3*0)*(cos(0) - 0.3*sin(0)); 
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
    
    % f(t) = 2*sin(t)*exp(-0.3*t)
    dx = 1;
    dy = 2*exp(-0.3*tk)*(cos(tk) - 0.3*sin(tk));
    ddy = 2*exp(-0.3*tk)*(-1.09*sin(tk) - 0.6*cos(tk));
    
    % Paso 4: Velocidad lineal u(t) [cite: 32, 37, 67]
    u(k) = sqrt(dx^2 + dy^2);
    
    % Paso 6: Velocidad angular w(t) [cite: 48, 59, 65]
    w(k) = ddy / (1 + dy^2);
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    tk = t(k);
        % Esto evita que el error se acumule y la trayectoria se desvíe [cite: 41, 42]
    dy_actual = 2*exp(-0.3*tk)*(cos(tk) - 0.3*sin(tk));
    phi(k+1) = atan2(dy_actual, 1); 
    
    % Modelo Cinemático Diferencial [cite: 82]
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
axis([-1 12 -1.5 1.5 0 2]); % Ejes ajustados para ver mejor la oscilación

scale = 2;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

step = 10; % Aumentamos el salto para que la animación no tarde tanto
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