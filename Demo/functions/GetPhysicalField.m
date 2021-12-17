function field = GetPhysicalField(pos,Blocker,Ori,m,n,a,b,c,precision)
if isequal(pos,Blocker)
    field = inf;
    return;
end
[CosA,SinA] = GetTriangle(pos,Blocker,Ori);
d = norm(Blocker - pos) * precision;
if d <= (a*b)/sqrt((a * CosA)^2 + (b * SinA)^2)
    field = inf;
else
    field = 0.01;
end
end

