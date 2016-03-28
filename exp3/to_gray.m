path_entrada ='db/seq3/';
path_salida='db/seq3/gray/';
vid_files = dir(strcat(path_entrada,'*.jpg'));
for i = 1:length(vid_files)
    disp(vid_files(i).name);
    A = imread(strcat(path_entrada,vid_files(i).name));
    B = rgb2gray(A);
    imwrite(B,strcat(path_salida,'out_',vid_files(i).name));
end