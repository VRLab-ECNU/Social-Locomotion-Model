function EnergyMap = GetMap(x,y,Blocker,Ori,m,n,a,b,c,precision)
%GetMap Get an Energy Map based on SIFM
EnergyMap = nan(x,y);

parfor i = 1:x
    for j = 1:y
        pos = [i-1,j-1];
        if isequal(pos,Blocker)
            EnergyMap(i,j) = inf;
            continue;
        end
        [CosA,SinA] = GetTriangle(pos,Blocker,Ori);
        if CosA < 0
            fA = 0;
        else
            fA = CosA;
        end
        d = norm(Blocker - pos) * precision;
        EnergyMap(i,j) = (m * fA + n + c * (a*b)/sqrt((a * CosA)^2 + (b * SinA)^2))/d^2;
    end
end
end

