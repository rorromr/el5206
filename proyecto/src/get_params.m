%% Este script permite obtener los parametros del modelo gaussiano
files = dir('db/db_prueba_target/*.jpg');
pixel_data = [];
non_pixel_data=[];
i=0;
for file = files'
    target = sprintf('db/db_prueba_target/%s',file.name);
    original = sprintf('db/db_prueba/%s',file.name);
    disp(['Target: ',target,' | Original: ', original]);
    [pixel, non_pixel]=get_pixel(original,target);
    pixel_data = [pixel_data; pixel];
    non_pixel_data = [non_pixel_data; non_pixel];
    i=i+1;
    if i>3
        break
    end
end
%% Calculo de parametros
[pixel_N,~] = size(pixel_data);
[non_pixel_N,~] = size(non_pixel_data);
N = pixel_N+non_pixel_N;
P_pixel = pixel_N/N;
P_non_pixel = non_pixel_N/N;

mean_pixel = mean(pixel_data);
std_pixel = std(pixel_data);

mean_non_pixel = mean(non_pixel_data);
std_non_pixel = std(non_pixel_data);



