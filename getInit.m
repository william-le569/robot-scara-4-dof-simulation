function [th,d,al,a,e]=getInit(handles)

th1=str2double(get(handles.th1,'String'))*pi/180;
th2=str2double(get(handles.th2,'String'))*pi/180;
d3=str2double(get(handles.d3,'String'));
th4=str2double(get(handles.th4,'String'))*pi/180;
th=[th1 th2 0 th4]';
a=[400 250 0 0]';
d=[347.25 0 d3 -30]';
al=[0 0 0 pi]';

e=[0 0 0 0;
50 50 -50 -50;
-50 0 0 -50;
1 1 1 1];