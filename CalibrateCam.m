clearvars;clc;
%% Camera calibration
% This script performs the extrinsic camera calibration to find the
% relative transformation matrix between the camera sensor frame A and the
% camera casing frame G. It uses the intrinsics obtained via an intrinsic
% calibration and uses the photo's and reconstructed poses in the
% "extrinsics" folder (obtained via VISP library). This script reads out
% the poses and computes the relative transformation. It is mainly based on
% the following two works:
% 
% [1] F. C. Park and B. J. Martin, "Robot sensor calibration: solving AX=XB
% on the Euclidean group," in IEEE Transactions on Robotics and Automation,
% vol. 10, no. 5, pp. 717-721, Oct. 1994, doi: 10.1109/70.326576.
%
% [2] R. Y. Tsai and R. K. Lenz, "A new technique for fully autonomous and 
% efficient 3D robotics hand/eye calibration," in IEEE Transactions on 
% Robotics and Automation, vol. 5, no. 3, pp. 345-358, June 1989,
% doi: 10.1109/70.34770.
%% Load data
fncsv = dir('data/*.csv');
fnpng = dir('data/*.png');


%Board-to-camera poses obtained from calibratecam.py
poses = table2array(readtable('camera_poses.csv'));
N = length(poses); %Number of images/frames we have

%Obtain the data from Optitrack
tel = 1;
for ii = 1:length(fncsv)
    if startsWith(fncsv(ii).name,"Take") && endsWith(fncsv(ii).name,".csv")
        data = OptImport(append(fncsv(ii).folder,'\',fncsv(ii).name));
        optitrack{1,tel} = data.POSTPROCESSING.RealSense002.transforms.ds;
        tel = tel+1;
    end
end

%Compute an average frame from first 200 data points
MH_G = zeros(4,4,N);
for ii = 1:N
    rotinit = optitrack{ii}{1}(1:3,1:3);
    for jj = 1:200
        frame(:,:,jj) = optitrack{ii}{jj};
    end
    MH_G(1:3,4,ii) = mean(squeeze(frame(1:3,4,1)),2);    
    MH_G(1:3,1:3,ii) = meanSO3(frame(1:3,1:3,:));
    MH_G(4,4,ii)=1;
end

%Obtain the poses from calibratecam.py
poses = table2array(readtable('camera_poses.csv'));
for ii = 1:length(poses)
    AH_B(:,:,ii) = [expm(hat(poses(ii,4:6))) poses(ii,1:3)'; zeros(1,3),1];
end



%% The following is based on [1]. See that paper for details
%First create the matrices A and B
tel = 1;
for ii=1:size(MH_G,3)
    for jj=ii+1:size(MH_G,3)
        A(:,:,tel)=(inv(MH_G(:,:,jj))*MH_G(:,:,ii));
        B(:,:,tel)=inv(AH_B(:,:,ii)*inv(AH_B(:,:,jj)));
        tel = tel+1;
    end
end

% Now find the rotation matrix
M=zeros(3,3);
for ii = 1:length(B)
    beta = logm(B(1:3,1:3,ii));
    alpha = logm(A(1:3,1:3,ii));
    M = M + vee(beta)*vee(alpha)';
end
GR_A = (M'*M)^(-0.5)*M';

% Using the found rotation matrix, we find the position using LS
C = eye(3)-A(1:3,1:3,1);
d = A(1:3,4,1)-GR_A*B(1:3,4,1);
for ii = 2:length(B)
    C = [C; eye(3)-A(1:3,1:3,ii)];
    d = [d; A(1:3,4,ii)-GR_A*B(1:3,4,ii);];
end
Go_A = (C'*C)\C'*d;

% Which combined gives us the final transformation matrix
GH_A = [GR_A Go_A; zeros(1,3),1];

%% Optimization to find the relative transformation
% Alternatively, one could uncomment the code below, which computes the
% relative transformation using MATLAB's lsqnonlin function

R_init = [1 0 0; 0 1 0; 0 0 1];
Theta0 = zeros(6,1);%[0 0 -0.01 rotm2eul(R_init,'XYZ')]';
fun = @(x)MyCalibCostFun(x, MH_G, AH_B);
Theta = lsqnonlin(fun,Theta0);
GH_A = [eul2rotm(Theta(4:6)','XYZ'),Theta(1:3); 0 0 0 1] ;

function F = MyCalibCostFun(vars, MH_G, AH_B)
F = [];

GH_A = [eul2rotm([vars(4),vars(5),vars(6)], 'XYZ') , vars(1:3); 0 0 0 1];
for ii=1:size(MH_G,3)
    for jj=ii+1:size(MH_G,3)
        H = inv(MH_G(:,:,jj))*MH_G(:,:,ii)*GH_A*AH_B(:,:,ii)*inv(AH_B(:,:,jj))*inv(GH_A);
        F = [F ; H(1:3,4) ; -rotationMatrixToVector(H(1:3,1:3))' ];
    end
end

end

%% Functions
function Rmean = meanSO3(R)

Npart = length(R);
Rmean = R(:,:,1); %Take  initial mean
    
% Map the other particles to a tangent space at wmean
for ii = 1:Npart
    RT(:,ii) = vee(logm(Rmean\R(:,:,ii)));
end

%Compute the mean in the tangent space, check if it is at the origin
Wmean = mean(RT,2);     
while norm(Wmean) > 1e-5 %If not at the origin, an update is executed
    Rmean = Rmean*expm(hat(Wmean));
    for ii = 1:Npart
        % Map the other particles to a tangent space at wmean
        RT(:,ii) = vee(logm(Rmean\R(:,:,ii)));
    end
    Wmean = mean(RT,2); 
end
end