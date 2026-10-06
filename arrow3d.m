 
function [h]=arrow3d(x,y,z,head_frac,radii,radii2,colr)

    if nargin==5
        radii2=radii*2;
        colr='blue';
    elseif nargin==6
        colr='blue';
    end;
    if size(x,1)==2
        x=x';
        y=y';
        z=z';
    end;
    x(3)=x(2);
    x(2)=x(1)+head_frac*(x(3)-x(1));
    y(3)=y(2);
    y(2)=y(1)+head_frac*(y(3)-y(1));
    z(3)=z(2);
    z(2)=z(1)+head_frac*(z(3)-z(1));
    r=[x(1:2)',y(1:2)',z(1:2)'];
    N=50;
    dr=diff(r);
    dr(end+1,:)=dr(end,:);
    origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
    r=r+origin_shift;
    normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
    normdr=[normdr,normdr,normdr];
    dr=dr./normdr;
    Pc=r;
    n1=cross(dr,Pc);
    normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
    normn1=[normn1,normn1,normn1];
    n1=n1./normn1;
    P1=n1+Pc;
    X1=[];Y1=[];Z1=[];
    j=1;
    for theta=([0:N])*2*pi./(N);
        R1=Pc+radii*cos(theta).*(P1-Pc) + radii*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
        X1(2:3,j)=R1(:,1);
        Y1(2:3,j)=R1(:,2);
        Z1(2:3,j)=R1(:,3);
        j=j+1;
    end
    r=[x(2:3)',y(2:3)',z(2:3)'];
    dr=diff(r);
    dr(end+1,:)=dr(end,:);
    origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
    r=r+origin_shift;
    normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
    normdr=[normdr,normdr,normdr];
    dr=dr./normdr;
    Pc=r;
    n1=cross(dr,Pc);
    normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
    normn1=[normn1,normn1,normn1];
    n1=n1./normn1;
    P1=n1+Pc;
    j=1;
    for theta=([0:N])*2*pi./(N);
        R1=Pc+radii2*cos(theta).*(P1-Pc) + radii2*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
        X1(4:5,j)=R1(:,1);
        Y1(4:5,j)=R1(:,2);
        Z1(4:5,j)=R1(:,3);
        j=j+1;
    end
    X1(1,:)=X1(1,:)*0 + x(1);
    Y1(1,:)=Y1(1,:)*0 + y(1);
    Z1(1,:)=Z1(1,:)*0 + z(1);
    X1(5,:)=X1(5,:)*0 + x(3);
    Y1(5,:)=Y1(5,:)*0 + y(3);
    Z1(5,:)=Z1(5,:)*0 + z(3);
    h=surf(X1,Y1,Z1,'facecolor',colr,'edgecolor','none');
    
    lighting phong