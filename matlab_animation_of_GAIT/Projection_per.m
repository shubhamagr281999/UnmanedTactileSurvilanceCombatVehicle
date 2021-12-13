function [xp,yp] = Projection_per(a,xc,yc,zc)
    x1= a(1);
    y1 = a(2);
    z1 = a(3);    
    xp = xc + (-zc/(z1-zc))*(x1 - xc);
    yp = yc + (-zc/(z1-zc))*(y1 - yc);      
end