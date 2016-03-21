function [ result ] = lbp(image_adress)
    % Load image
    raw_image = imread(image_adress);
    % Check gray scale
    if(length(size(raw_image)) ~= 2)
        raw_image = rgb2gray(raw_image);
    end
    % Get size
    [M,N] = size(raw_image);
    lbp_img = zeros(M,N);
    % LBP weight
    weight = [1 2 4; 128 0 8; 64 32 16];
    for i = 2:M-1
        for j = 2:N-1
            mask = zeros(3,3);
            for mi = 1:3
                for mj = 1:3
                    if(raw_image(i-(2-mi),j-(2-mj)) >= raw_image(i,j))
                        mask(mi,mj) = weight(mi,mj);
                    end
                end
            end
    
            lbp_img(i,j) = sum(sum(mask));
        end
    end
    lbp_img = uint8(lbp_img);
    result = lbp_img;
end
