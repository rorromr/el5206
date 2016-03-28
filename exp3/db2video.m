function [ video ] = db2video( path )
%DB2VIDEO 
vid_files = dir(strcat(path,'*.jpg'));
length(vid_files)
video = cell(length(vid_files),1);

for i = 1:length(vid_files)
    video{i} = imread(strcat(path,vid_files(i).name));
end

end

