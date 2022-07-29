function EnergyMap = GetMap(x,y,Blocker,Ori,Target,m1,n1,m2,n2,a,b,c,precision,ncf)
%GetMap Get an Energy Map based on SIFM
EnergyMap = nan(x,y);

parfor i = 1:x
    for j = 1:y
        pos = [i-1,j-1];
        if isequal(pos,Blocker)
            EnergyMap(i,j) = inf;
            continue;
        end
        if isequal(pos,Target)
            myfield = n2;
        else
            MyOri = acosd(sum((pos-Target).*[0,-1])/(norm(pos-Target)))+180;
            [CosB,SinB] = GetTriangle(Blocker,pos,MyOri);
            
            if CosB < 0
                fB = 0;
            else
                fB = CosB;
            end
            
            myfield = m2 * fB + n2 + c * (a*b)/sqrt((a * CosB)^2 + (b * SinB)^2);
        end
        [CosA,SinA] = GetTriangle(pos,Blocker,Ori);
        
        if CosA < 0
            fA = 0;
        else
            fA = CosA;
        end

        d = norm(Blocker - pos) * precision;
        hisfield = m1 * fA + n1 + c * (a*b)/sqrt((a * CosA)^2 + (b * SinA)^2);
        EnergyMap(i,j) = (hisfield * myfield)/d^ncf;
end
end

