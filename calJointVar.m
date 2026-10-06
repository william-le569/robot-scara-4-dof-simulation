function J=calJointVar(T)
    J0=[0 0 0 1]';
    J=zeros(4,4);
    for i=1:4
         J(:,i)  = T(:,:,i)*J0;
    end;