function [CosA,SinA] = GetTriangle(posA,posB,oriB)
%GetTriangle Calculate the relative angle between A & B, given their
%position and orientation
posC = posB + [sind(oriB),-cosd(oriB)];
Vector1 = posA - posB;
Vector2 = posC - posB;
CosA = sum(Vector1.*Vector2)/(norm(Vector1)*norm(Vector2));
SinA = sqrt(1-CosA^2);
end

