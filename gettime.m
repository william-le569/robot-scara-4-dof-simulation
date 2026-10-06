function [t1,t2,t3]=gettime(tf,tc,step)
t1=0:step:tc;
t2=tc:step:tf-tc;
t31=tf-tc:step:tf-3*step;
t32=tf-3*step:step/3:tf;
t3=[t31 t32];
