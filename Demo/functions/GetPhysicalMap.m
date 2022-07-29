function EnergyMap = GetPhysicalMap(x,y,Blocker,Ori,m,n,a,b,c,precision)
%GetMap Get an Energy Map based on SIFM
EnergyMap = 0.01*ones(x,y);
p1 = Blocker(1);
p2 = Blocker(2);

for i = floor(p1-a/precision)+1:ceil(p1+a/precision)+1
    for j = floor(p2-a/precision)+1:ceil(p2+a/precision)+1
        if i<1 || i>x || j<1 || j>y
            continue;
        end
        pos = [i-1,j-1];
        if isequal(pos,Blocker)
            EnergyMap(i,j) = inf;
            continue;
        end
        [CosA,SinA] = GetTriangle(pos,Blocker,Ori);
        d = norm(Blocker - pos) * precision;
        if d < (a*b)/sqrt((a * CosA)^2 + (b * SinA)^2)
            EnergyMap(i,j) = inf;
        end
    end
end
end

