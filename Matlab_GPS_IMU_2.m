%Borrar previos
delete(instrfind({'Port'},{'COM12'}));
%Crear objeto serie
s = serial('COM12','BaudRate',115200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
%Abrir puerto
fopen(s);

%Initial values
x = [0; 0]; % Initial values
accelx_1 = 0;
heading = 0;
% Noise
Q = [1e-6 0; 0 1e-5]; % Noise of the proccess (how much idea I have of the proccess). Process noise covariance
R = [1 0; 0 1]; % Noise of the measurements (sensor). Measurement noise covariance. 1000 no confías nada. 0 confias
P = [1 0; 0 1]; % Error covariance
I = [1 0; 0 1]; % Identity matrix

while true
data = fscanf(s,'%f,%f,%f,%f'); % Recieve data
dt = data(4);

% PREDICTION____________________________________________________________
% Position x Prediction
% State space representation
A = [1 dt; 0 1];
B = [(0.5*(dt^2)); dt];
u = accelx_1;
x = A*x+B*u % Estado. Vector (porque hay una multiplicación) 

H = [0 0; 0 1]; % What I can measure
z_1 = H*x; % Sensor prediction
P = A*P*(inv(A))+Q; % Current error covariance 

% UPDATE______________________________________________________________

heading = data(2);
accelx_1 = data(3); 
speedx_1 = data(1)*cos(heading); 

z = [0; speedx_1];
y = z-z_1;
S = H*P*(inv(H))+R;
K = P*(inv(H))*(inv(S));
x = x+K*y %Esta es la correcta
P = (I-K*H)*P;

end %Ctrl + c to end the program