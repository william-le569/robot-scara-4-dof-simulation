function varargout = Scara_Robot(varargin)
% SCARA_ROBOT MATLAB code for Scara_Robot.fig
%      SCARA_ROBOT, by itself, creates a new SCARA_ROBOT or raises the existing
%      singleton*.
%
%      H = SCARA_ROBOT returns the handle to a new SCARA_ROBOT or the handle to
%      the existing singleton*.
%
%      SCARA_ROBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SCARA_ROBOT.M with the given input arguments.
%
%      SCARA_ROBOT('Property','Value',...) creates a new SCARA_ROBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Scara_Robot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Scara_Robot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Scara_Robot

% Last Modified by GUIDE v2.5 30-Dec-2020 02:11:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Scara_Robot_OpeningFcn, ...
                   'gui_OutputFcn',  @Scara_Robot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Scara_Robot is made visible.
function Scara_Robot_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Scara_Robot (see VARARGIN)

% Choose default command line output for Scara_Robot
handles.output = hObject;
guidata(hObject, handles);
handles.output = hObject;
guidata(hObject, handles);
   
    hold off
    th=[0  pi/2 0 0]';
    a=[400 250 0 0]';
    d=[347.25 0 -50 -30]';
    al=[0 0 0 pi]';
    e=[0 0 0 0;
    50 50 -50 -50;
    -50 0 0 -50;
    1 1 1 1];

    T=HomoTransform(th,d,al,a);
    J=calJointVar(T);
    e=T(:,:,3)*e;
    
    % Euler equations... / Reference page 50 ....
    yaw  = atan2(T(2,1,4),T(1,1,4)); % Rotate about Ox...
    pitch= atan2(-T(3,1,4),sqrt(T(3,2,4)^2+T(3,3,4)^2)); %Rotate about Oy...
    roll = atan2(T(3,2,4),T(3,3,4));  % Rotate about Oz...
    
    %[ Function displayJointVar(th1,th2,d3,th4,handles) just to display...
    displayJointVar(0,pi/2,-50,0,handles);
    displayEE(J,handles);
    handles.roll.String=num2str(roll*180/pi);
    handles.pitch.String=num2str(pitch*180/pi);
    handles.yaw.String=num2str(yaw*180/pi);

    axes(handles.axes1);
    drawPose(J,e);

    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');



% --- Executes on button press in btn_Forward.
function btn_Forward_Callback(~, ~, handles)
    hold off;
    [th,d,al,a,e]=getInit(handles);
    T=HomoTransform(th,d,al,a);
    J=calJointVar(T);
    e=T(:,:,3)*e;
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);

    pitch= atan2(-T(3,1,4),sqrt(T(3,2,4)^2+T(3,3,4)^2));
    roll = atan2(T(3,2,4),T(3,3,4));
    yaw  = atan2(T(2,1,4),T(1,1,4)); 

    handles.roll.String=num2str(roll*180/pi);
    handles.pitch.String=num2str(pitch*180/pi);
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);

    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');




% --- Executes on button press in btn_Inverse.
function btn_Inverse_Callback(~, ~, handles)
    hold off;
   
    posx = str2double(get(handles.posx,'String'));  
    posy = str2double(get(handles.posy,'String'));
    posz = str2double(get(handles.posz,'String'));
    yaw  = str2double(get(handles.yaw,'String'))*pi/180;

    [th,d,al,a,]=getInit(handles);

    [th_new,d_new]=InvKinematic([posx posy posz],yaw,th,d);
jump=40;
step=Qstep([th(1) th(2) d(3) th(4)],[th_new(1) th_new(2) d_new(3) th_new(4)],jump);
th(1)=th(1)-step(1);th(2)=th(2)-step(2);d(3)=d(3)-step(3);th(4)=th(4)-step(4);
path=zeros(3,length(11));
 for i=1:jump+1
     hold off;
     e=[0 0 0 0;
        50 50 -50 -50;
        -50 0 0 -50;
        1 1 1 1];
     th(1)=th(1)+step(1);th(2)=th(2)+step(2);
     d(3)=d(3)+step(3);th(4)=th(4)+step(4);
     T=HomoTransform(th,d,al,a);
     J=calJointVar(T);
     e=T(:,:,3)*e;
     drawPose(J,e);
     drawAxis(J,T);
 
     path(:,i)=J(1:3,4);
     plot3(path(1,1:i),path(2,1:i),path(3,1:i),'y','LineWidth',2);
       displayJointVar(th(1),th(2),d(3),th(4),handles);
       displayEE(J,handles);
     grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(10^-20);
   
 end
 function step=Qstep(q_old,q_new,jump)
m=(q_new(1)-q_old(1))/jump;
n=(q_new(2)-q_old(2))/jump;
p=(q_new(3)-q_old(3))/jump;
k=(q_new(4)-q_old(4))/jump;
step=[m n p k]';






% --- Executes on button press in btn_Trajectory_Planning.
function btn_Trajectory_Planning_Callback(~, ~, handles)
    qmax= str2double(get(handles.qmax,'String'));
    vmax= str2double(get(handles.vmax,'String'));
    amax= str2double(get(handles.amax,'String'));
    hold off;
    tc=vmax/amax;
    tf=qmax/vmax+tc;
    
    [t1,t2,t3]=gettime(tf,tc,0.01);
    t=[t1 t2 t3];
    sigma1=vmax/2/tc*t1.^2; % (vmax/(2*tc))*t1...
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
 
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    a1=ones(1,length(t1))*vmax/tc;
    a2=zeros(1,length(t2));
    a3=-ones(1,length(t3))*vmax/tc;
    
    sigma_2dot=[a1 a2 a3];
    
    figure;
    subplot(3,1,1)
    plot(t,sigma,'b');xlabel('t(s)');ylabel('position(cm)');grid on;
    subplot(3,1,2)
    plot(t,sigma_dot,'b');xlabel('t(s)');ylabel('velocity(cm)');grid on;
    subplot(3,1,3)
    plot(t,sigma_2dot,'b');xlabel('t(s)');ylabel('acceleration(cm)');grid on;
    




% --- Executes on button press in btn_Linear.
function btn_Linear_Callback(~, ~, handles)
    hold off;

    posx_new = str2double(get(handles.posx,'String'));
    posy_new = str2double(get(handles.posy,'String'));
    posz_new = str2double(get(handles.posz,'String'));
    yaw      = str2double(get(handles.yaw,'String'))*pi/180;
    
    vmax= str2double(get(handles.vmax,'String'));
    amax= str2double(get(handles.amax,'String'));
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    
    p0=[J(1,4) J(2,4) J(3,4)]';%old position % toa do x,y,z cua Joint 4
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
    tf=qmax/vmax+tc;
    [t1,t2,t3]=gettime(tf,tc,0.3);
    t=[t1 t2 t3];
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
    q=zeros(4,length(p(1,:)));
    path=zeros(3,length(p(1,:)));
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);
        q(:,i)=[th(1) th(2) d(3) th(4)]';
        e=T(:,:,3)*e;

        path(:,i)=J(1:3,4);

        
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
        plot3(path(1,1:i),path(2,1:i),path(3,1:i),'g','LineWidth',2);
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.001);
    end
    vyaw=zeros(1,length(s));
    vyaw(1)=0;
    for i=2:length(yaw)
        vyaw(i)=(yaw(i)-yaw(i-1))/(t(i)-t(i-1));
    end;
    ve=[p_dot;vyaw];
    q_dot=zeros(4,length(s));
    for i=1:length(s)
        th=[q(1,i) q(2,i) 0 q(4,i)]';
        invJ=invJacobian(th,a);
        q_dot(:,i)=invJ*ve(:,i);
    end;
    handles.qmax.String=num2str(qmax);

    figure;
    subplot(4,1,1);
    plot(t,q_dot(1,:),'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('J1(rad/s)');
    subplot(4,1,2);
    plot(t,q_dot(2,:),'r','LineWidth',1);grid on;xlabel('t(s)');ylabel('J2(rad/s)');
    subplot(4,1,3);
    plot(t,q_dot(3,:),'y','LineWidth',1);grid on;xlabel('t(s)');ylabel('J3(mm/s)');
    subplot(4,1,4);
    plot(t,q_dot(4,:)*180/pi,'g','LineWidth',1);grid on;xlabel('t(s)');ylabel('J4(Deg/s)');

    figure;
    title('Actual Joint motion');
    plot(t,q(1,:)*180/pi,'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('Position of joint');hold on;
    plot(t,q(2,:)*180/pi,'r','LineWidth',1);hold on;
    plot(t,q(3,:),'y','LineWidth',1);hold on;
    plot(t,q(4,:)*180/pi,'g','LineWidth',1);xlabel('t(s)');
    
    legend('th1(Deg)','th2(Deg)','d3(mm)','th4(Deg)');
    phi_dot=zeros(1,length(s));
    phi_dot(1)=0;
    for i=2:length(s)
        phi_dot(i)=(phi(i)-phi(i-1))/(t(i)-t(i-1));
    end;
    
    figure;
    subplot(4,1,1)
    plot(t,p_dot(1,:),'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('vx(mm/s)');
    subplot(4,1,2)
    plot(t,p_dot(2,:),'r','LineWidth',1);grid on;xlabel('t(s)');ylabel('vy(mm/s)');
    subplot(4,1,3)
    plot(t,p_dot(3,:),'y','LineWidth',1);grid on;xlabel('t(s)');ylabel('vz(mm/s)');
    subplot(4,1,4)
    plot(t,phi_dot*180/pi,'g','LineWidth',1);grid on;xlabel('t(s)');ylabel('vR(Deg/s)');
    
    figure;
    subplot(4,1,1)
    plot(t,p(1,:),'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('x(mm)');
    subplot(4,1,2)
    plot(t,p(2,:),'r','LineWidth',1);grid on;xlabel('t(s)');ylabel('y(mm)');
    subplot(4,1,3)
    plot(t,p(3,:),'y','LineWidth',1);grid on;xlabel('t(s)');ylabel('z(mm)');
    subplot(4,1,4)
    plot(t,phi*180/pi,'g','LineWidth',1);grid on;xlabel('t(s)');ylabel('R(Deg)');
    
 


% --- Executes on button press in btc_Circular.
function btc_Circular_Callback(~, ~, handles)
    hold off;


    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw  = str2double(get(handles.yaw,'String'))*pi/180;

    vmax= str2double(get(handles.vmax,'String'));
    amax= str2double(get(handles.amax,'String'));
    x0=str2double(get(handles.x0,'String'));
    y0=str2double(get(handles.y0,'String'));
    dir=str2double(get(handles.dir,'String'));

    [th,d,al,a]=getInit(handles);

    T=HomoTransform(th,d,al,a);
    J=calJointVar(T);
    
    r=sqrt((x0-J(1,4))^2+(y0-J(2,4))^2);
    phi_start=atan2(J(2,4)-y0,J(1,4)-x0);
    phi_end=atan2(y-y0,x-x0);
    delta=phi_end-phi_start;
    if dir*delta>0
        delta=dir*delta;
    else  
        delta=2*pi + dir*delta;
    end;

    qmax=delta*r;
    
    p0=[J(1,4) J(2,4) J(3,4)]';
    pf=[x y z]';
    tc=vmax/amax;
    tf=qmax/vmax+tc;
    [t1,t2,t3]=gettime(tf,tc,0.3);
    t=[t1 t2 t3];
    %POSITION TRAJCECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
     p_dot=zeros(3,length(s));
    p=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=x0+r*cos(phi_start+dir*sigma(i)/r);
        p(2,i)=y0+r*sin(phi_start+dir*sigma(i)/r);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(-1)*dir*sigma_dot(i)*sin(phi_start+dir*sigma(i)/r);
        p_dot(2,i)=dir*sigma_dot(i)*cos(phi_start+dir*sigma(i)/r);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    [th_new,d_new]=InvKinematic([x y z 1]',yaw,th,d);
    
    %ORIENTATION TRAJCECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    phi=zeros(1,length(s));
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    if(phif ~=0)
     r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
         (Rif(1,3)-Rif(3,1))/2/sin(phif);
         (Rif(2,1)-Rif(1,2))/2/sin(phif)];
        yaw=zeros(1,length(s));
        for i=1:length(s)
            phi(i)=s(i)*phif;
         RRR=AxisAngle(phi(i),r);
         R=Ri*RRR;
         yaw(i)=atan2(R(2,1),R(1,1));
        end;
    else
         yaw=angle*ones(1,length(s));
    end;
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
    q=zeros(4,length(p(1,:)));
    path=zeros(3,length(p(1,:)));
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);
        q(:,i)=[th(1) th(2) d(3) th(4)]';
        e=T(:,:,3)*e;

        path(:,i)=J(1:3,4);

        
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
        plot3(path(1,1:i),path(2,1:i),path(3,1:i),'g','LineWidth',2);
        
        
        handles.posx.String=num2str(J(1,4));
        handles.posy.String=num2str(J(2,4));
        handles.posz.String=num2str(J(3,4));
       
        handles.th1.String=num2str(th(1)*180/pi);
        handles.th2.String=num2str(th(2)*180/pi);
        handles.th4.String=num2str(th(4)*180/pi);
        handles.d3.String=num2str(d(3));

        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
    vyaw=zeros(1,length(s));
    vyaw(1)=0;
    for i=2:length(q(4,:))
        vyaw(i)=(yaw(i)-yaw(i-1))/(t(i)-t(i-1));
    end;
    ve=[p_dot;vyaw];
    q_dot=zeros(4,length(s));
    for i=1:length(s)
        th=[q(1,i) q(2,i) 0 q(4,i)]';
        invJ=invJacobian(th,a);
        q_dot(:,i)=invJ*ve(:,i);
    end;
    handles.qmax.String=num2str(qmax);
    figure;
    subplot(4,1,1);
    plot(t,q_dot(1,:),'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('J1(rad/s)');
    subplot(4,1,2);
    plot(t,q_dot(2,:),'r','LineWidth',1);grid on;xlabel('t(s)');ylabel('J2(rad/s)');
    subplot(4,1,3);
    plot(t,q_dot(3,:),'y','LineWidth',1);grid on;xlabel('t(s)');ylabel('J3(mm/s)');
    subplot(4,1,4);
    plot(t,q_dot(4,:)*180/pi,'g','LineWidth',1);grid on;xlabel('t(s)');ylabel('J4(Deg/s)');

    figure;
    title('Actual Joint motion');
    plot(t,q(1,:)*180/pi,'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('Position of joint');hold on;
    plot(t,q(2,:)*180/pi,'r','LineWidth',1);hold on;
    plot(t,q(3,:),'y','LineWidth',1);hold on;
    plot(t,q(4,:)*180/pi,'g','LineWidth',1);xlabel('t(s)');
    
    legend('th1(Deg)','th2(Deg)','d3(mm)','th4(Deg)');
    phi_dot=zeros(1,length(s));
    for i=2:length(s)
        phi_dot(i)=(phi(i)-phi(i-1))/(t(i)-t(i-1));
    end;
    
    figure;
    subplot(4,1,1)
    plot(t,p_dot(1,:),'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('vx(mm/s)');
    subplot(4,1,2)
    plot(t,p_dot(2,:),'r','LineWidth',1);grid on;xlabel('t(s)');ylabel('vy(mm/s)');
    subplot(4,1,3)
    plot(t,p_dot(3,:),'y','LineWidth',1);grid on;xlabel('t(s)');ylabel('vz(mm/s)');
    subplot(4,1,4)
    plot(t,phi_dot*180/pi,'g','LineWidth',1);grid on;xlabel('t(s)');ylabel('vR(Deg/s)');
    
    figure;
    subplot(4,1,1)
    plot(t,p(1,:),'b','LineWidth',1);grid on;xlabel('t(s)');ylabel('x(mm)');
    subplot(4,1,2)
    plot(t,p(2,:),'r','LineWidth',1);grid on;xlabel('t(s)');ylabel('y(mm)');
    subplot(4,1,3)
    plot(t,p(3,:),'y','LineWidth',1);grid on;xlabel('t(s)');ylabel('z(mm)');
    subplot(4,1,4)
    plot(t,phi*180/pi,'g','LineWidth',1);grid on;xlabel('t(s)');ylabel('R(Deg)');
    
    
    
% --- Executes on button press in btn_positive_theta1.
function btn_positive_theta1_Callback(~, ~, handles)
   
    step=str2double(get(handles.step,'String'))*pi/180;
    hold off;
    [th,d,al,a,e]=getInit(handles);
    th(1)=th(1)+step;
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    e=T(:,:,3)*e;
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);
    
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
    
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');




% --- Executes on button press in btn_negative_theta1.
function btn_negative_theta1_Callback(~, ~, handles)
    step=str2double(get(handles.step,'String'))*pi/180;
    hold off;
    [th,d,al,a,e]=getInit(handles);
    th(1)=th(1)-step;
    T=HomoTransform(th,d,al,a);

    J=calJointVar(T);
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    e=T(:,:,3)*e;
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);

    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');



% --- Executes on button press in btn_positive_ttheta2.
function btn_positive_ttheta2_Callback(~, ~, handles)
    step=str2double(get(handles.step_th2,'String'))*pi/180;
    hold off;
    [th,d,al,a,e]=getInit(handles);
    th(2)=th(2)+step;

    T=HomoTransform(th,d,al,a);
    J=calJointVar(T);
    e=T(:,:,3)*e;
    yaw  = atan2(T(2,1,4),T(1,1,4)); 

    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);

    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
    
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');


% --- Executes on button press in btn_negative_theta2.
function btn_negative_theta2_Callback(~, ~, handles)
    step=str2double(get(handles.step_th2,'String'))*pi/180;
    hold off;
    [th,d,al,a,e]=getInit(handles);
    th(2)=th(2)-step;
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    e=T(:,:,3)*e;
    
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);
    
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
   
 
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

% --- Executes on button press in btn_positive_d3.
function btn_positive_d3_Callback(~, ~, handles)
    step=str2double(get(handles.step_d3,'String'));
    hold off;
    [th,d,al,a,e]=getInit(handles);
    d(3)=d(3)+step;
    
    T=HomoTransform(th,d,al,a);

    J=calJointVar(T);
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    e=T(:,:,3)*e;
    
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);
    
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
    
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');


% --- Executes on button press in btn_Negative_d3.
function btn_Negative_d3_Callback(~, ~, handles)
    step=str2double(get(handles.step_d3,'String'));
    hold off;
    [th,d,al,a,e]=getInit(handles);
    d(3)=d(3)-step;
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    e=T(:,:,3)*e;
    
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);
    
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
    
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

% --- Executes on button press in btn_positive_theta4.
function btn_positive_theta4_Callback(~, ~, handles)
    step=str2double(get(handles.step_th4,'String'))*pi/180;
    hold off;
    [th,d,al,a,e]=getInit(handles);
    th(4)=th(4)+step;
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    e=T(:,:,3)*e;
    
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);
    
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
   
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
% --- Executes on button press in btn_negative_theta4.
function btn_negative_theta4_Callback(~, ~, handles)
    step=str2double(get(handles.step_th4,'String'))*pi/180;
    hold off;

    [th,d,al,a,e]=getInit(handles);
    th(4)=th(4)-step;
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    yaw  = atan2(T(2,1,4),T(1,1,4)); 
    e=T(:,:,3)*e;
    
    axes(handles.axes1);
    drawPose(J,e);
    drawAxis(J,T);
    
    handles.yaw.String=num2str(yaw*180/pi);
    displayEE(J,handles);
    displayJointVar(th(1),th(2),d(3),th(4),handles);
    
    
    grid on;
    axis([-1000 1000 -1000 1000 0 500]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

    % --- Executes on button press in btn_get_position_0.
function btn_get_position_0_Callback(~, ~, handles)
    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw0 = str2double(get(handles.yaw,'String'));

    handles.posx0.String=num2str(x);
    handles.posy0.String=num2str(y);
    handles.posz0.String=num2str(z);
    handles.use0.String=num2str(1);
    handles.yaw0.String=num2str(yaw0);



% --- Executes on button press in btn_get_position_1.
function btn_get_position_1_Callback(~, ~, handles)
    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw1 = str2double(get(handles.yaw,'String'));

    handles.yaw1.String=num2str(yaw1);
    handles.posx1.String=num2str(x);
    handles.posy1.String=num2str(y);
    handles.posz1.String=num2str(z);
    handles.use1.String=num2str(1);

% --- Executes on button press in btn_get_position_2.
function btn_get_position_2_Callback(~, ~, handles)
    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw2 = str2double(get(handles.yaw,'String'));

    handles.yaw2.String=num2str(yaw2);
    handles.posx2.String=num2str(x);
    handles.posy2.String=num2str(y);
    handles.posz2.String=num2str(z);
    handles.use2.String=num2str(1);

% --- Executes on button press in btn_get_position_3.
function btn_get_position_3_Callback(~, ~, handles)
    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw3 = str2double(get(handles.yaw,'String'));

    handles.yaw3.String=num2str(yaw3);
    handles.posx3.String=num2str(x);
    handles.posy3.String=num2str(y);
    handles.posz3.String=num2str(z);
    handles.use3.String=num2str(1);


% --- Executes on button press in btn_get_position_4.
function btn_get_position_4_Callback(~, ~, handles)
    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw4 = str2double(get(handles.yaw,'String'));

    handles.yaw4.String=num2str(yaw4);
    handles.posx4.String=num2str(x);
    handles.posy4.String=num2str(y);
    handles.posz4.String=num2str(z);
    handles.use4.String=num2str(1);


% --- Executes on button press in btn_get_position_5.
function btn_get_position_5_Callback(~, ~, handles)
    x = str2double(get(handles.posx,'String'));
    y = str2double(get(handles.posy,'String'));
    z = str2double(get(handles.posz,'String'));
    yaw5 = str2double(get(handles.yaw,'String'));

    handles.yaw5.String=num2str(yaw5);
    handles.posx5.String=num2str(x);
    handles.posy5.String=num2str(y);
    handles.posz5.String=num2str(z);
    handles.use5.String=num2str(1);


% --- Executes on button press in btn_home.
function btn_home_Callback(hObject, eventdata, handles)
hold off;

    posx_new = 400;
    posy_new = 250;
    posz_new = 267.25;
    yaw  = pi/2;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=8;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
    path=zeros(3,length(p(1,:)));
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
% --- Executes on button press in btn_pick_and_place.
function btn_pick_and_place_Callback(~, ~, handles)
    use0 = str2double(get(handles.use0,'String'));
    use1 = str2double(get(handles.use1,'String'));
    use2 = str2double(get(handles.use2,'String'));
    use3 = str2double(get(handles.use3,'String'));
    use4 = str2double(get(handles.use4,'String'));
    use5 = str2double(get(handles.use5,'String'));
   

   hold off;
    
    posx_new = 400;
    posy_new = 250;
    posz_new = 267.25;
    yaw  = pi/2;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=5;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
    path=zeros(3,length(p(1,:)));
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        F=get(handles.F0,'String');
        
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
       
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        grid on;
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end

    if (use0==1)
        home2pos0(handles);
    end;

    if (use1==1)
        pos02pos1(handles);
    end;

    if (use2==1)
        pos12pos2(handles);
    end;
    
    if (use3==1)
       pos22pos3(handles);
    
    end;

    if (use4==1)
        pos32pos4(handles);
    end;

    if (use5==1)
       pos42pos5(handles);
    end;

  
function home2pos0(handles)
    hold off;

    posx_new = str2double(get(handles.posx0,'String'));
    posy_new = str2double(get(handles.posy0,'String'));
    posz_new = str2double(get(handles.posz0,'String'));
    yaw  = str2double(get(handles.yaw0,'String'))*pi/180;
 

    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
    path=zeros(3,length(p(1,:)));
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
      
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
   
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
 
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
function pos02pos1(handles)
    hold off;

    posx_new = str2double(get(handles.posx1,'String'));
    posy_new = str2double(get(handles.posy1,'String'));
    posz_new = str2double(get(handles.posz1,'String'));
    yaw  = str2double(get(handles.yaw1,'String'))*pi/180;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
  
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        F=get(handles.F0,'String');
        if (F=='1')
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
     
        else
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
        end;
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
    
function pos12pos2(handles)
    hold off;

    posx_new = str2double(get(handles.posx2,'String'));
    posy_new = str2double(get(handles.posy2,'String'));
    posz_new = str2double(get(handles.posz2,'String'));
    yaw  = str2double(get(handles.yaw2,'String'))*pi/180;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
  
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        F=get(handles.F1,'String');
        if (F=='1')
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
     
        else
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
        end;
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);hold on;
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        drawAxis(J,T);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
    
function pos22pos3(handles)
    hold off;

    posx_new = str2double(get(handles.posx3,'String'));
    posy_new = str2double(get(handles.posy3,'String'));
    posz_new = str2double(get(handles.posz3,'String'));
    yaw  = str2double(get(handles.yaw3,'String'))*pi/180;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
  
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        F=get(handles.F2,'String');
        if (F=='1')
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
     
        else
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
        end;
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
    
function pos32pos4(handles)
    hold off;

    posx_new = str2double(get(handles.posx4,'String'));
    posy_new = str2double(get(handles.posy4,'String'));
    posz_new = str2double(get(handles.posz4,'String'));
    yaw  = str2double(get(handles.yaw4,'String'))*pi/180;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
  
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        F=get(handles.F3,'String');
        if (F=='1')
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
     
        else
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
        end;
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
function pos42pos5(handles)
    hold off;

    posx_new = str2double(get(handles.posx5,'String'));
    posy_new = str2double(get(handles.posy5,'String'));
    posz_new = str2double(get(handles.posz5,'String'));
    yaw  = str2double(get(handles.yaw5,'String'))*pi/180;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
  
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        F=get(handles.F4,'String');
        if (F=='1')
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
     
        else
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
        end;
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end
function pos52pos6(handles)
    hold off;

    posx_new = str2double(get(handles.posx2,'String'));
    posy_new = str2double(get(handles.posy2,'String'));
    posz_new = str2double(get(handles.posz2,'String'));
    yaw  = str2double(get(handles.yaw2,'String'))*pi/180;
    [th,d,al,a,~]=getInit(handles);
    
    T=HomoTransform(th,d,al,a);
    
    J=calJointVar(T);
    
    qmax=sqrt((J(1,4)-posx_new)^2+(J(2,4)-posy_new)^2+(J(3,4)-posz_new)^2);
    tf=3;
    vmax=2*qmax/tf*0.8;
    amax=vmax^2/(-qmax+tf*vmax);
    p0=[J(1,4) J(2,4) J(3,4)]';%old position
    
    pf=[posx_new posy_new posz_new]';%new position
    
    [th_new,d_new]=InvKinematic([posx_new posy_new posz_new 1]',yaw,th,d);
    tc=vmax/amax;
   
    [t1,t2,t3]=gettime(tf,tc,0.2);
    
    %POSITION TRACJECTORY
    sigma1=vmax/2/tc*t1.^2;
    sigma2=vmax*(t2-tc/2);
    sigma3=qmax-vmax*(tf-t3).^2/2/tc;
    sigma=[sigma1 sigma2 sigma3];
 
    s=sigma/qmax;
    
    sigma_dot1=vmax/tc*t1;
    sigma_dot2=ones(1,length(t2))*vmax;
    sigma_dot3=vmax/tc*(tf-t3);
    sigma_dot=[sigma_dot1 sigma_dot2 sigma_dot3];
    
    s_dot=sigma_dot/qmax;
    
    p=zeros(3,length(s));
     p_dot=zeros(3,length(s));
    for i=1:length(s)
        p(1,i)=p0(1)+(pf(1)-p0(1))*s(i);
        p(2,i)=p0(2)+(pf(2)-p0(2))*s(i);
        p(3,i)=p0(3)+(pf(3)-p0(3))*s(i);
        
        p_dot(1,i)=(pf(1)-p0(1))*s_dot(i);
        p_dot(2,i)=(pf(2)-p0(2))*s_dot(i);
        p_dot(3,i)=(pf(3)-p0(3))*s_dot(i);
    end;
    
    %ORIENTATION TRACJECTORY
    angle=yaw;
    Ti=T;
    Tf=HomoTransform(th_new,d_new,al,a);
    Ri=zeros(3,3);
    Rf=zeros(3,3);
    Ri(:,:)=Ti(1:3,1:3,4);
    Rf(:,:)=Tf(1:3,1:3,4);
    Rif=Ri'*Rf;
    phif=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);
    phi=zeros(1,length(s));
    if (phif~=0)
    r=[(Rif(3,2)-Rif(2,3))/2/sin(phif);
        (Rif(1,3)-Rif(3,1))/2/sin(phif);
        (Rif(2,1)-Rif(1,2))/2/sin(phif)];
    
    yaw=zeros(1,length(s));
    for i=1:length(s)
        phi(i)=s(i)*phif;
        RRR=AxisAngle(phi(i),r);
        R=Ri*RRR;
        yaw(i)=atan2(R(2,1),R(1,1)); 
    end;
    else
        yaw=angle*ones(1,length(s));
    end;
  
    %DRAW MOTION OF ROBOT ON AXES
    axes(handles.axes1);
   
  
    for i=1:length(s)
        hold off;
 
        [th,d]=InvKinematic([p(1,i) p(2,i) p(3,i)],yaw(i),th,d);
        e=[0 0 0 0;50 50 -50 -50;-50 0 0 -50;1 1 1 1];
      
        T=HomoTransform(th,d,al,a);
        J=calJointVar(T);

        e=T(:,:,3)*e;
        displayJointVar(th(1),th(2),d(3),th(4));
        drawPose(J,e);
       
        
        displayEE(J,handles);
        displayJointVar(th(1),th(2),d(3),th(4),handles);
        handles.yaw.String=num2str(yaw(i)*180/pi);
        drawAxis(J,T);
        F=get(handles.F5,'String');
        if (F=='1')
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
     
        else
        x=str2double(get(handles.x,'String'));
        y=str2double(get(handles.y,'String'));
        z=str2double(get(handles.z,'String'));
     
        end;
        plot3(x,y,z,'o','MarkerEdgeColor','g','LineWidth',8);
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);
        grid on;
        axis([-1000 1000 -1000 1000 0 500]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        pause(0.0001);
    end

function drawAxis(J,T)
    % calculate J4 axis
    J4_xaxis=T(:,:,4)*[300 0 0 1]';
    J4_yaxis=T(:,:,4)*[0 300 0 1]';
    J4_zaxis=T(:,:,4)*[0 0 100 1]';
    hold on;
    arrow3d([J(1,4) J4_xaxis(1)],[J(2,4) J4_xaxis(2)],[J(3,4) J4_xaxis(3)],0.5,3,5,'r');
    arrow3d([J(1,4) J4_yaxis(1)],[J(2,4) J4_yaxis(2)],[J(3,4) J4_yaxis(3)],0.5,3,5,'b');
    arrow3d([J(1,4) J4_zaxis(1)],[J(2,4) J4_zaxis(2)],[J(3,4) J4_zaxis(3)],0.5,9,15,'y');
    %calculate J2 axis
    J2_xaxis=T(:,:,2)*[300 0 0 1]';
    J2_yaxis=T(:,:,2)*[0 300 0 1]';
    J2_zaxis=T(:,:,2)*[0 0 100 1]';
    arrow3d([J(1,2) J2_xaxis(1)],[J(2,2) J2_xaxis(2)],[J(3,2) J2_xaxis(3)],0.5,3,5,'r');
    arrow3d([J(1,2) J2_yaxis(1)],[J(2,2) J2_yaxis(2)],[J(3,2) J2_yaxis(3)],0.5,3,5,'b');
    arrow3d([J(1,2) J2_zaxis(1)],[J(2,2) J2_zaxis(2)],[J(3,2) J2_zaxis(3)],0.5,9,15,'y');
    %calculate J1 axis
    J1_xaxis=T(:,:,1)*[300 0 0 1]';
    J1_yaxis=T(:,:,1)*[0 300 0 1]';
    J1_zaxis=T(:,:,1)*[0 0 100 1]';
    arrow3d([J(1,1) J1_xaxis(1)],[J(2,1) J1_xaxis(2)],[J(3,1) J1_xaxis(3)],0.5,3,5,'r');
    arrow3d([J(1,1) J1_yaxis(1)],[J(2,1) J1_yaxis(2)],[J(3,1) J1_yaxis(3)],0.5,3,5,'b');
    arrow3d([J(1,1) J1_zaxis(1)],[J(2,1) J1_zaxis(2)],[J(3,1) J1_zaxis(3)],0.5,9,15,'y');
    %calculate frame 0 axis
    frame0_xaxis=[300 0 0 1]';
    frame0_yaxis=[0 300 0 1]';
    frame0_zaxis=[0 0 100 1]';
    arrow3d([0 frame0_xaxis(1)],[0 frame0_xaxis(2)],[0 frame0_xaxis(3)],0.5,3,5,'r');
    arrow3d([0 frame0_yaxis(1)],[0 frame0_yaxis(2)],[0 frame0_yaxis(3)],0.5,3,5,'b');
    arrow3d([0 frame0_zaxis(1)],[0 frame0_zaxis(2)],[0 frame0_zaxis(3)],0.5,9,15,'y');


    

function drawPose(J,e)
    J0=[0 0 0 1]';
    plot3([J0(1)    0    J(1,1) J(1,2) J(1,3) ],...
           [J0(2)    0    J(2,1) J(2,2) J(2,3) ],...
        [J0(3) 347.25  J(3,1) J(3,2) J(3,3) ],'b',...
        'LineWidth',5);hold on;
 
    plot3([e(1,1) e(1,2) e(1,3) e(1,4)],...
        [e(2,1) e(2,2) e(2,3) e(2,4)],...
        [e(3,1) e(3,2) e(3,3) e(3,4)],'b',...
        'LineWidth',5);

    plot3(0,0,0,'o','MarkerEdgeColor','r','LineWidth',8);hold on;
    plot3(0,0,347.25,'o','MarkerEdgeColor','r','LineWidth',8);hold on;
    for i=1:3
    plot3(J(1,i),J(2,i),J(3,i),'o','MarkerEdgeColor','r','LineWidth',8);
    end;
    
function R=AxisAngle(v,r)
    rx=r(1);
    ry=r(2);
    rz=r(3);
    R=[rx^2*(1-cos(v))+cos(v),          rx*ry*(1-cos(v))-rz*sin(v),      rx*rz*(1-cos(v))+ry*sin(v);
   rx*ry*(1-cos(v))+rz*sin(v),      ry^2*(1-cos(v))+cos(v)    ,      ry*rz*(1-cos(v))-rx*sin(v);
   rx*rz*(1-cos(v))-ry*sin(v),      ry*rz*(1-cos(v))+rx*sin(v),      rz^2*(1-cos(v))+cos(v) ];




% Display parameters of posx,posy,posz
function displayEE(J,handles)
    handles.posx.String=num2str(J(1,4));
    handles.posy.String=num2str(J(2,4));
    handles.posz.String=num2str(J(3,4));
 
    % Display joint parameters...
function displayJointVar(th1,th2,d3,th4,handles)
    handles.th1.String=num2str(th1*180/pi);
    handles.th2.String=num2str(th2*180/pi);
    handles.th4.String=num2str(th4*180/pi);
    handles.d3.String=num2str(d3);
    
    
function invJ=invJacobian(th,a)
invJ =[ -cos(th(1) + th(2))/(a(1)*sin(th(2))),  -sin(th(1) + th(2))/(a(1)*sin(th(2))),  0,  0;
(a(2)*cos(th(1) + th(2)) + a(1)*cos(th(1)))/(a(1)*a(2)*sin(th(2))), (a(2)*sin(th(1) + th(2)) + a(1)*sin(th(1)))/(a(1)*a(2)*sin(th(2))), 0, 0;
                                                  0,                                                  0, 1, 0;
                            -cos(th(1))/(a(2)*sin(th(2))),                            -sin(th(1))/(a(2)*sin(th(2))), 0, 1];
                        





function use0_Callback(hObject, eventdata, handles)
% hObject    handle to use0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use0 as text
%        str2double(get(hObject,'String')) returns contents of use0 as a double


% --- Executes during object creation, after setting all properties.
function use0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use1_Callback(hObject, eventdata, handles)
% hObject    handle to use1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use1 as text
%        str2double(get(hObject,'String')) returns contents of use1 as a double


% --- Executes during object creation, after setting all properties.
function use1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use2_Callback(hObject, eventdata, handles)
% hObject    handle to use2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use2 as text
%        str2double(get(hObject,'String')) returns contents of use2 as a double


% --- Executes during object creation, after setting all properties.
function use2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use3_Callback(hObject, eventdata, handles)
% hObject    handle to use3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use3 as text
%        str2double(get(hObject,'String')) returns contents of use3 as a double


% --- Executes during object creation, after setting all properties.
function use3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use4_Callback(hObject, eventdata, handles)
% hObject    handle to use4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use4 as text
%        str2double(get(hObject,'String')) returns contents of use4 as a double


% --- Executes during object creation, after setting all properties.
function use4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx5_Callback(hObject, eventdata, handles)
% hObject    handle to posx5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx5 as text
%        str2double(get(hObject,'String')) returns contents of posx5 as a double


% --- Executes during object creation, after setting all properties.
function posx5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx6_Callback(hObject, eventdata, handles)
% hObject    handle to posx6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx6 as text
%        str2double(get(hObject,'String')) returns contents of posx6 as a double


% --- Executes during object creation, after setting all properties.
function posx6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx7_Callback(hObject, eventdata, handles)
% hObject    handle to posx7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx7 as text
%        str2double(get(hObject,'String')) returns contents of posx7 as a double


% --- Executes during object creation, after setting all properties.
function posx7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx8_Callback(hObject, eventdata, handles)
% hObject    handle to posx8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx8 as text
%        str2double(get(hObject,'String')) returns contents of posx8 as a double


% --- Executes during object creation, after setting all properties.
function posx8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx9_Callback(hObject, eventdata, handles)
% hObject    handle to posx9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx9 as text
%        str2double(get(hObject,'String')) returns contents of posx9 as a double


% --- Executes during object creation, after setting all properties.
function posx9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy5_Callback(hObject, eventdata, handles)
% hObject    handle to posy5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy5 as text
%        str2double(get(hObject,'String')) returns contents of posy5 as a double


% --- Executes during object creation, after setting all properties.
function posy5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy6_Callback(hObject, eventdata, handles)
% hObject    handle to posy6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy6 as text
%        str2double(get(hObject,'String')) returns contents of posy6 as a double


% --- Executes during object creation, after setting all properties.
function posy6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy7_Callback(hObject, eventdata, handles)
% hObject    handle to posy7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy7 as text
%        str2double(get(hObject,'String')) returns contents of posy7 as a double


% --- Executes during object creation, after setting all properties.
function posy7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy8_Callback(hObject, eventdata, handles)
% hObject    handle to posy8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy8 as text
%        str2double(get(hObject,'String')) returns contents of posy8 as a double


% --- Executes during object creation, after setting all properties.
function posy8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy9_Callback(hObject, eventdata, handles)
% hObject    handle to posy9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy9 as text
%        str2double(get(hObject,'String')) returns contents of posy9 as a double


% --- Executes during object creation, after setting all properties.
function posy9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz5_Callback(hObject, eventdata, handles)
% hObject    handle to posz5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz5 as text
%        str2double(get(hObject,'String')) returns contents of posz5 as a double


% --- Executes during object creation, after setting all properties.
function posz5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz6_Callback(hObject, eventdata, handles)
% hObject    handle to posz6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz6 as text
%        str2double(get(hObject,'String')) returns contents of posz6 as a double


% --- Executes during object creation, after setting all properties.
function posz6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz7_Callback(hObject, eventdata, handles)
% hObject    handle to posz7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz7 as text
%        str2double(get(hObject,'String')) returns contents of posz7 as a double


% --- Executes during object creation, after setting all properties.
function posz7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz8_Callback(hObject, eventdata, handles)
% hObject    handle to posz8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz8 as text
%        str2double(get(hObject,'String')) returns contents of posz8 as a double


% --- Executes during object creation, after setting all properties.
function posz8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz9_Callback(hObject, eventdata, handles)
% hObject    handle to posz9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz9 as text
%        str2double(get(hObject,'String')) returns contents of posz9 as a double


% --- Executes during object creation, after setting all properties.
function posz9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use5_Callback(hObject, eventdata, handles)
% hObject    handle to use5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use5 as text
%        str2double(get(hObject,'String')) returns contents of use5 as a double


% --- Executes during object creation, after setting all properties.
function use5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use6_Callback(hObject, eventdata, handles)
% hObject    handle to use6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use6 as text
%        str2double(get(hObject,'String')) returns contents of use6 as a double


% --- Executes during object creation, after setting all properties.
function use6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use7_Callback(hObject, eventdata, handles)
% hObject    handle to use7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use7 as text
%        str2double(get(hObject,'String')) returns contents of use7 as a double


% --- Executes during object creation, after setting all properties.
function use7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use8_Callback(hObject, eventdata, handles)
% hObject    handle to use8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use8 as text
%        str2double(get(hObject,'String')) returns contents of use8 as a double


% --- Executes during object creation, after setting all properties.
function use8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function use9_Callback(hObject, eventdata, handles)
% hObject    handle to use9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of use9 as text
%        str2double(get(hObject,'String')) returns contents of use9 as a double


% --- Executes during object creation, after setting all properties.
function use9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to use9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function step_Callback(hObject, eventdata, handles)
% hObject    handle to step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of step as text
%        str2double(get(hObject,'String')) returns contents of step as a double


% --- Executes during object creation, after setting all properties.
function step_CreateFcn(hObject, eventdata, handles)
% hObject    handle to step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function step_th2_Callback(hObject, eventdata, handles)
% hObject    handle to step_th2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of step_th2 as text
%        str2double(get(hObject,'String')) returns contents of step_th2 as a double


% --- Executes during object creation, after setting all properties.
function step_th2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to step_th2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function step_d3_Callback(hObject, eventdata, handles)
% hObject    handle to step_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of step_d3 as text
%        str2double(get(hObject,'String')) returns contents of step_d3 as a double


% --- Executes during object creation, after setting all properties.
function step_d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to step_d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function step_th4_Callback(hObject, eventdata, handles)
% hObject    handle to step_th4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of step_th4 as text
%        str2double(get(hObject,'String')) returns contents of step_th4 as a double


% --- Executes during object creation, after setting all properties.
function step_th4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to step_th4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function posx1_Callback(hObject, eventdata, handles)
% hObject    handle to posx1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx1 as text
%        str2double(get(hObject,'String')) returns contents of posx1 as a double


% --- Executes during object creation, after setting all properties.
function posx1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy1_Callback(hObject, eventdata, handles)
% hObject    handle to posy1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy1 as text
%        str2double(get(hObject,'String')) returns contents of posy1 as a double


% --- Executes during object creation, after setting all properties.
function posy1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz1_Callback(hObject, eventdata, handles)
% hObject    handle to posz1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz1 as text
%        str2double(get(hObject,'String')) returns contents of posz1 as a double


% --- Executes during object creation, after setting all properties.
function posz1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx2_Callback(hObject, eventdata, handles)
% hObject    handle to posx2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx2 as text
%        str2double(get(hObject,'String')) returns contents of posx2 as a double


% --- Executes during object creation, after setting all properties.
function posx2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx3_Callback(hObject, eventdata, handles)
% hObject    handle to posx3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx3 as text
%        str2double(get(hObject,'String')) returns contents of posx3 as a double


% --- Executes during object creation, after setting all properties.
function posx3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx4_Callback(hObject, eventdata, handles)
% hObject    handle to posx4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx4 as text
%        str2double(get(hObject,'String')) returns contents of posx4 as a double


% --- Executes during object creation, after setting all properties.
function posx4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy2_Callback(hObject, eventdata, handles)
% hObject    handle to posy2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy2 as text
%        str2double(get(hObject,'String')) returns contents of posy2 as a double


% --- Executes during object creation, after setting all properties.
function posy2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy3_Callback(hObject, eventdata, handles)
% hObject    handle to posy3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy3 as text
%        str2double(get(hObject,'String')) returns contents of posy3 as a double


% --- Executes during object creation, after setting all properties.
function posy3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy4_Callback(hObject, eventdata, handles)
% hObject    handle to posy4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy4 as text
%        str2double(get(hObject,'String')) returns contents of posy4 as a double


% --- Executes during object creation, after setting all properties.
function posy4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz2_Callback(hObject, eventdata, handles)
% hObject    handle to posz2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz2 as text
%        str2double(get(hObject,'String')) returns contents of posz2 as a double


% --- Executes during object creation, after setting all properties.
function posz2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz3_Callback(hObject, eventdata, handles)
% hObject    handle to posz3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz3 as text
%        str2double(get(hObject,'String')) returns contents of posz3 as a double


% --- Executes during object creation, after setting all properties.
function posz3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz4_Callback(hObject, eventdata, handles)
% hObject    handle to posz4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz4 as text
%        str2double(get(hObject,'String')) returns contents of posz4 as a double


% --- Executes during object creation, after setting all properties.
function posz4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function yaw0_Callback(hObject, eventdata, handles)
% hObject    handle to yaw0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw0 as text
%        str2double(get(hObject,'String')) returns contents of yaw0 as a double


% --- Executes during object creation, after setting all properties.
function yaw0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw1_Callback(hObject, eventdata, handles)
% hObject    handle to yaw1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw1 as text
%        str2double(get(hObject,'String')) returns contents of yaw1 as a double


% --- Executes during object creation, after setting all properties.
function yaw1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw2_Callback(hObject, eventdata, handles)
% hObject    handle to yaw2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw2 as text
%        str2double(get(hObject,'String')) returns contents of yaw2 as a double


% --- Executes during object creation, after setting all properties.
function yaw2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw3_Callback(hObject, eventdata, handles)
% hObject    handle to yaw3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw3 as text
%        str2double(get(hObject,'String')) returns contents of yaw3 as a double


% --- Executes during object creation, after setting all properties.
function yaw3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw4_Callback(hObject, eventdata, handles)
% hObject    handle to yaw4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw4 as text
%        str2double(get(hObject,'String')) returns contents of yaw4 as a double


% --- Executes during object creation, after setting all properties.
function yaw4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw5_Callback(hObject, eventdata, handles)
% hObject    handle to yaw5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw5 as text
%        str2double(get(hObject,'String')) returns contents of yaw5 as a double


% --- Executes during object creation, after setting all properties.
function yaw5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw6_Callback(hObject, eventdata, handles)
% hObject    handle to yaw6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw6 as text
%        str2double(get(hObject,'String')) returns contents of yaw6 as a double


% --- Executes during object creation, after setting all properties.
function yaw6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw7_Callback(hObject, eventdata, handles)
% hObject    handle to yaw7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw7 as text
%        str2double(get(hObject,'String')) returns contents of yaw7 as a double


% --- Executes during object creation, after setting all properties.
function yaw7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw8_Callback(hObject, eventdata, handles)
% hObject    handle to yaw8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw8 as text
%        str2double(get(hObject,'String')) returns contents of yaw8 as a double


% --- Executes during object creation, after setting all properties.
function yaw8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw9_Callback(hObject, eventdata, handles)
% hObject    handle to yaw9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw9 as text
%        str2double(get(hObject,'String')) returns contents of yaw9 as a double


% --- Executes during object creation, after setting all properties.
function yaw9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Outputs from this function are returned to the command line.
function varargout = Scara_Robot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function th1_Callback(hObject, eventdata, handles)
% hObject    handle to th1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of th1 as text
%        str2double(get(hObject,'String')) returns contents of th1 as a double


% --- Executes during object creation, after setting all properties.
function th1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to th1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function th2_Callback(hObject, eventdata, handles)
% hObject    handle to th2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of th2 as text
%        str2double(get(hObject,'String')) returns contents of th2 as a double


% --- Executes during object creation, after setting all properties.
function th2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to th2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d3_Callback(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d3 as text
%        str2double(get(hObject,'String')) returns contents of d3 as a double


% --- Executes during object creation, after setting all properties.
function d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function th4_Callback(hObject, eventdata, handles)
% hObject    handle to th4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of th4 as text
%        str2double(get(hObject,'String')) returns contents of th4 as a double


% --- Executes during object creation, after setting all properties.
function th4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to th4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posx_Callback(hObject, eventdata, handles)
% hObject    handle to posx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx as text
%        str2double(get(hObject,'String')) returns contents of posx as a double


% --- Executes during object creation, after setting all properties.
function posx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy_Callback(hObject, eventdata, handles)
% hObject    handle to posy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy as text
%        str2double(get(hObject,'String')) returns contents of posy as a double


% --- Executes during object creation, after setting all properties.
function posy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz_Callback(hObject, eventdata, handles)
% hObject    handle to posz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz as text
%        str2double(get(hObject,'String')) returns contents of posz as a double


% --- Executes during object creation, after setting all properties.
function posz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as a double


% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitch_Callback(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitch as text
%        str2double(get(hObject,'String')) returns contents of pitch as a double


% --- Executes during object creation, after setting all properties.
function pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw_Callback(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw as text
%        str2double(get(hObject,'String')) returns contents of yaw as a double


% --- Executes during object creation, after setting all properties.
function yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function qmax_Callback(hObject, eventdata, handles)
% hObject    handle to qmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of qmax as text
%        str2double(get(hObject,'String')) returns contents of qmax as a double


% --- Executes during object creation, after setting all properties.
function qmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vmax_Callback(hObject, eventdata, handles)
% hObject    handle to vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vmax as text
%        str2double(get(hObject,'String')) returns contents of vmax as a double


% --- Executes during object creation, after setting all properties.
function vmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function amax_Callback(hObject, eventdata, handles)
% hObject    handle to amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of amax as text
%        str2double(get(hObject,'String')) returns contents of amax as a double


% --- Executes during object creation, after setting all properties.
function amax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to amax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function x0_Callback(hObject, eventdata, handles)
% hObject    handle to x0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x0 as text
%        str2double(get(hObject,'String')) returns contents of x0 as a double


% --- Executes during object creation, after setting all properties.
function x0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y0_Callback(hObject, eventdata, handles)
% hObject    handle to y0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y0 as text
%        str2double(get(hObject,'String')) returns contents of y0 as a double


% --- Executes during object creation, after setting all properties.
function y0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dir_Callback(hObject, eventdata, handles)
% hObject    handle to dir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dir as text
%        str2double(get(hObject,'String')) returns contents of dir as a double


% --- Executes during object creation, after setting all properties.
function dir_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function posx0_Callback(hObject, eventdata, handles)
% hObject    handle to posx0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posx0 as text
%        str2double(get(hObject,'String')) returns contents of posx0 as a double


% --- Executes during object creation, after setting all properties.
function posx0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posx0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posy0_Callback(hObject, eventdata, handles)
% hObject    handle to posy0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posy0 as text
%        str2double(get(hObject,'String')) returns contents of posy0 as a double


% --- Executes during object creation, after setting all properties.
function posy0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posy0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function posz0_Callback(hObject, eventdata, handles)
% hObject    handle to posz0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of posz0 as text
%        str2double(get(hObject,'String')) returns contents of posz0 as a double


% --- Executes during object creation, after setting all properties.
function posz0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to posz0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tf_Callback(hObject, eventdata, handles)
% hObject    handle to tf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tf as text
%        str2double(get(hObject,'String')) returns contents of tf as a double


% --- Executes during object creation, after setting all properties.
function tf_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tf (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





   



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double


% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
 hold off;
        x=str2double(get(handles.posx,'String'));
        y=str2double(get(handles.posy,'String'));
        z=str2double(get(handles.posz,'String'));
        handles.x.String=num2str(x);
        handles.y.String=num2str(y);
        handles.z.String=num2str(z);









function F0_Callback(hObject, eventdata, handles)
% hObject    handle to F0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of F0 as text
%        str2double(get(hObject,'String')) returns contents of F0 as a double


% --- Executes during object creation, after setting all properties.
function F0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to F0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function F1_Callback(hObject, eventdata, handles)
% hObject    handle to F1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of F1 as text
%        str2double(get(hObject,'String')) returns contents of F1 as a double


% --- Executes during object creation, after setting all properties.
function F1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to F1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function F2_Callback(hObject, eventdata, handles)
% hObject    handle to F2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of F2 as text
%        str2double(get(hObject,'String')) returns contents of F2 as a double


% --- Executes during object creation, after setting all properties.
function F2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to F2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function F3_Callback(hObject, eventdata, handles)
% hObject    handle to F3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of F3 as text
%        str2double(get(hObject,'String')) returns contents of F3 as a double


% --- Executes during object creation, after setting all properties.
function F3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to F3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function F4_Callback(hObject, eventdata, handles)
% hObject    handle to F4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of F4 as text
%        str2double(get(hObject,'String')) returns contents of F4 as a double


% --- Executes during object creation, after setting all properties.
function F4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to F4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function F5_Callback(hObject, eventdata, handles)
% hObject    handle to F5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of F5 as text
%        str2double(get(hObject,'String')) returns contents of F5 as a double


% --- Executes during object creation, after setting all properties.
function F5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to F5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function btn_Linear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to btc_Circular (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



% --- Executes during object creation, after setting all properties.
function btc_Circular_CreateFcn(hObject, eventdata, handles)
% hObject    handle to btc_Circular (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

function btn_Forward_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to btn_Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
