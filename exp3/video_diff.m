function [ diff_frame ] = video_diff( video, thr )
%VIDEO_DIFF
diff_frame = cell(length(video)-1,1);

for i=1:length(diff_frame)
    diff_frame{i} = im2bw(video{i+1} - video{i}, thr);
    imshow(diff_frame{i}), drawnow
end


end

