function T=HomoTransform(th,d,al,a)
    A=zeros(4,4,4);
    for i=1:4 
        A(:,:,i)=[ cos(th(i))   -cos(al(i))*sin(th(i))   sin(al(i))*sin(th(i))    a(i)*cos(th(i));
                   sin(th(i))    cos(al(i))*cos(th(i))  -sin(al(i))*cos(th(i))    a(i)*sin(th(i));
                       0               sin(al(i))               cos(al(i))              d(i)     ;
                       0                   0                        0                    1      ];
     end;
 
    T=zeros(4,4,4);
    T(:,:,1)=A(:,:,1);
    for i=2:4
        T(:,:,i)=T(:,:,i-1)*A(:,:,i);
    end;