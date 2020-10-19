% Written by Muhammet Balcilar, France
% All rights reserved

clear all;
%close all
clc;

image_2_path = 'data/image_2/';
calib_path = 'data/calib/';
bin_path = 'data/velodyne/';
File = dir(fullfile(image_2_path,'*.png'));  % 显示文件夹下所有符合后缀名为.png文件的完整信息
FileNames = {File.name}';                    % 提取符合后缀名为.txt的所有文件的文件名，转换为n行1列
FileNumbers = size(FileNames);
FileNumbers = FileNumbers(1);

for i=1:FileNumbers
    file_name = char(FileNames(i));
    temp_ = strsplit(file_name,'.');
    file_id = char(temp_(1));
end
