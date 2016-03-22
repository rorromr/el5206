function [ d ] = vector_d( v1, v2, distance_type )
    if distance_type == 1
        sum = 0;
        for i = 1:length(v1)
            sum = sum + abs(v1(i) - v2(i));
        end
        d = sum;
    else
        sum = 0;
        for i = 1:length(v1)
            sum = sum + sqrt(((v1(i) - v2(i))^2));
        end
        d = sum;
    end
end

