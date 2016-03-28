function [ out ] = motion_detector_blob( video, mean_bg, std_bg, thr )
%MOTION_DETECTOR_BLOB
out = motion_detector(video, mean_bg, std_bg, thr);
[rows, cols] = size(video{1});
% Estimator
C = [0.1, 0.1, 0.1, 0.1];
P = zeros(length(out),4); % parameters
pnew = ones(1,4);
for i=2:length(out)-1
    imshow(video{i}), hold on
    rows_hist = sum(out{i}) / 256;
    cols_hist = sum(out{i},2) / 256;
    dx = 1500*std(cols_hist);
    [~,x] = max(cols_hist);
    dy = 1500*std(rows_hist);
    [~,y] = max(rows_hist);
    P(i,1:4)=[y-dy/2,x-dx/2,dy,dx];
    P(i+1,1:4)=P(i,1:4)+C*(P(i,1:4)-P(i-1,1:4))';
    rectangle('Position',P(i,1:4)','LineWidth',3,'EdgeColor','r');
    rectangle('Position',pnew','LineWidth',3,'EdgeColor','b');
    
    axis([1 cols 1 rows]);
    drawnow
    hold off
    pnew = abs(P(i+1,1:4)+ones(1,4));
    pause(0.1);
end
close all;

end

