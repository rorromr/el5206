%% Cargar modelo gaussiano
global P_non_pixel P_pixel mean_non_pixel std_non_pixel mean_pixel std_pixel
load('params.mat')

%% Ejemplo de clasificacion
% El algoritmo tarda demasiado, se recomienda el uso de imagenes de baja
% resolucion
b=image_classify('db602p.jpg');
% Mostrar imagen
imshow(b)
% Guardar imagen binarizada
imwrite(b,'test_classify.jpg')

%% Ejemplo de binarizacion
m = compare_bin('db602p_target.jpg','test_classify.jpg')