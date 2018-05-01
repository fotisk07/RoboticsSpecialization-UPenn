%%%%
function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
while 1==1
    flag = true;
    p1=P1';
    p2=P2';
    a1=p1(:,1);
    b1=p1(:,2);
    c1=p1(:,3);
    a2=p2(:,1);
    b2=p2(:,2);
    c2=p2(:,3);
    
    %First Triangle
    v=a1-b1;
    v=[-v(2);v(1)];
    s=sign(v'*(a1-c1));
    s1=sign(v'*(a1-a2));
    s2=sign(v'*(a1-b2));
    s3=sign(v'*(a1-c2));
    if (s1 ~= s) && (s2~=s) && (s3~=s)
        flag=false;
        break;
    end
    
    v=b1-c1;
    v=[-v(2);v(1)];
    s=sign(v'*(b1-a1));
    s1=sign(v'*(b1-a2));
    s2=sign(v'*(b1-b2));
    s3=sign(v'*(b1-c2));
    if (s1 ~= s) && (s2~=s) && (s3~=s)
        flag=false;
        break;
    end
    
    v=c1-a1;
    v=[-v(2);v(1)];
    s=sign(v'*(c1-b1));
    s1=sign(v'*(c1-a2));
    s2=sign(v'*(c1-b2));
    s3=sign(v'*(c1-c2));
    if (s1 ~= s) && (s2~=s) && (s3~=s)
        flag=false; break;
    end
    
    
    %Second Triangle
    
    v=a2-b2;
    v=[-v(2);v(1)];
    s=sign(v'*(a2-c2));
    s1=sign(v'*(a2-a1));
    s2=sign(v'*(a2-b1));
    s3=sign(v'*(a2-c1));
    if (s1 ~= s) && (s2~=s) && (s3~=s)
        flag=false; break;
    end
    
    v=b2-c2;
    v=[-v(2);v(1)];
    s=sign(v'*(b2-a2));
    s1=sign(v'*(b2-a1));
    s2=sign(v'*(b2-b1));
    s3=sign(v'*(b2-c1));
    if (s1 ~= s) && (s2~=s) && (s3~=s)
        flag=false; break;
    end
    
    v=c2-a2;
    v=[-v(2);v(1)];
    s=sign(v'*(c2-b2));
    s1=sign(v'*(c2-a1));
    s2=sign(v'*(c2-b1));
    s3=sign(v'*(c2-c1));
    if (s1 ~= s) && (s2~=s) && (s3~=s)
        flag=false;break;
    end
    break;
end


% *******************************************************************
end