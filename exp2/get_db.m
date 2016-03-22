function [ db ] = get_db( folder )
    names = get_files(folder);
    disp(names);
    db = [];
    for i=1:length(names)
       lbp_img = lbp(names{i});
       v = get_vector(lbp_img);
       db = [db; v];
    end
end

