function [ bin_image ] = image_classify( img )
global P_non_pixel P_pixel mean_non_pixel std_non_pixel mean_pixel std_pixel
theta = 1;
a=imread(img);
[M,N,~]=size(a);
bin_image = zeros(M,N);
for i = 1:M
   for j = 1:N
       pixel = double([a(i,j,1) a(i,j,2) a(i,j,3)]);
       if mvnpdf(pixel, mean_pixel,diag(std_pixel))*P_pixel/(mvnpdf(pixel, mean_non_pixel,diag(std_non_pixel))*P_non_pixel)<theta
        bin_image(i,j)=1;
       end
   end   
end

end

