function [ out ] = motion_detector( video, mean, std, thr )
%MOTION_DETECTOR
m1 = mean - thr*std;
m2 = mean + thr*std;
out = cell(length(video));
for i=1:length(video)
    out{i} = (video{i} < m1) + (video{i} > m2);
end

end

