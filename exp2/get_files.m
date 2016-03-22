function [ file_list ] = get_files( folder )
    dir_data = dir(folder);
    dir_index = [dir_data.isdir];
    file_list = {dir_data(~dir_index).name}';
    for i=1:length(file_list)
        file_list{i} = strcat(folder,'/',file_list{i});
    end
end

