function alpha = GetAlpha(a)
%GetAlpha Get an angle given a 2-d vector
l = norm(a);
if a(2) >= 0
    alpha = acosd(-a(1)/l);
else
    alpha = -acosd(-a(1)/l);
end
end

