function [ mean_bg, std_bg ] = backgroud_model( video )
%BACKGROUD_MODEL
[rows,cols] = size(double(video{1}));
N = length(video);
mean_bg = zeros(rows,cols);
std_bg = zeros(rows,cols);
% Calc mean
for i=1:N
    mean_bg = mean_bg + double(video{i});
end
mean_bg = 1/N*mean_bg;
% Calc std
for i=1:N
    std_bg = std_bg + (mean_bg-double(video{i})).^2;
end
std_bg = (1/N*std_bg).^(0.5);

end

