function [ qhat, theta, u ] = Quat2u ( Q )
% this function used to get position vector (x,y,z) from Quaternions transformation matrix
%% Coded by
% Mohamed Mohamed El-Sayed Atyya
% mohamed.atyya94@eng-st.cu.edu.eg
%% inputs
% Q   :  Quaternions transformation matrix from XYZ to xyz
%% outputs
% qhat    :  The quaternion is the vector
% theta   :  Euler principal rotation angle in degree
% u          :  the Euler axis
% -----------------------------------------------------------------------------------------------------------------------------------------------------------
K=[Q(1,1)-Q(2,2)-Q(3,3), Q(2,1)+Q(1,2), Q(3,1)+Q(1,3), Q(2,3)-Q(3,2); ...
      Q(2,1)+Q(1,2), -Q(1,1)+Q(2,2)-Q(3,3), Q(3,2)+Q(2,3), Q(3,1)-Q(1,3); ...
      Q(3,1)+Q(1,3), Q(3,2)+Q(2,3), -Q(1,1)-Q(2,2)+Q(3,3), Q(1,2)-Q(2,1); ...
      Q(2,3)-Q(3,2), Q(3,1)-Q(1,3), Q(1,2)-Q(2,1), Q(1,1)+Q(2,2)+Q(3,3)]*1/3
[Eig_vectors, Eig_values]=eig(K)
[Max_Eig_values, index]=max(diag(Eig_values))
qhat=Eig_vectors(:,index)
theta=2*acosd(qhat(4))
u=qhat(1:end-1)/sind(theta/2)
end