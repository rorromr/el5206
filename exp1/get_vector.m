function [ out_vector ] = get_vector( lbp_img )
    % Cut image
    sub_img = lbp_img(2:501,2:601);
    % Create windows
    windows = mat2cell(sub_img, [100 100 100 100 100],[100 100 100 100 100 100]);
    out_vector = [];
    for i = 1:5
        for j = 1:6
            % Calc histogram
            temp_hist = hist(double(windows{i,j}(:)),59);
            out_vector = [out_vector temp_hist];
        end
    end
end

