function [ pixel, non_pixel ] = get_pixel( img, img_bin )
    a=imread(img);
    b=~boolean(imread(img_bin));
    T=sum(b(:));
    [M,N,~]=size(a);
    pixel = zeros(T,3);
    non_pixel = zeros(M*N-T,3);
    pixel_i = 1;
    non_pixel_i = 1;
    for i = 1:M
       for j = 1:N
           if b(i,j)==1
             pixel(pixel_i,:) = [a(i,j,1),a(i,j,2),a(i,j,3)];
             pixel_i = pixel_i + 1;
           else
             non_pixel(non_pixel_i,:) = [a(i,j,1),a(i,j,2),a(i,j,3)];
             non_pixel_i = non_pixel_i + 1;
           end
       end   
    end
end

