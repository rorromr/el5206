function [ m ] = compare_bin( b1, b2 )
%COMPARE_BIN
a=boolean(imread(b1));
b=boolean(imread(b2));
interseccion = a&b;
union = a|b;
m = sum(interseccion(:))/sum(union(:));
end

